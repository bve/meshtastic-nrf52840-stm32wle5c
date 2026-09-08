#include "configuration.h"

#if defined(USE_RIPU_STM32WL_RADIO)

#include "RipuSTM32WLBootloaderClient.h"

#include "SPILock.h"
#include "Throttle.h"

#include <cstring>

using namespace ripu_stm32wl;

RipuSTM32WLBootloaderClient::RipuSTM32WLBootloaderClient(SPIClass &spi, SPISettings spiSettings, int csPin, int busyPin,
                                                         int resetPin)
    : spi_(spi), spiSettings_(spiSettings), csPin_(csPin), busyPin_(busyPin), resetPin_(resetPin)
{
}

bool RipuSTM32WLBootloaderClient::enter()
{
    concurrency::LockGuard guard(spiLock);

    pinMode(csPin_, OUTPUT);
    pinMode(busyPin_, INPUT);
    pinMode(resetPin_, OUTPUT);
    spi_.begin();

    digitalWrite(csPin_, LOW);
    digitalWrite(resetPin_, LOW);
    delay(10);
    digitalWrite(resetPin_, HIGH);
    delay(200);
    digitalWrite(csPin_, HIGH);
    delay(10);

    sequence_ = 0;
    return waitReady(kReadyTimeoutMs);
}

void RipuSTM32WLBootloaderClient::resetApplication()
{
    concurrency::LockGuard guard(spiLock);

    digitalWrite(csPin_, HIGH);
    digitalWrite(resetPin_, LOW);
    delay(10);
    digitalWrite(resetPin_, HIGH);
    delay(100);
}

bool RipuSTM32WLBootloaderClient::getInfo(RipuBootloaderInfo &info, Status &status)
{
    uint8_t response[12] = {};
    size_t responseLength = 0;
    if (!command(Opcode::BootInfo, nullptr, 0, response, sizeof(response), responseLength, status)) {
        return false;
    }
    if (status == Status::Ok && responseLength != sizeof(response)) {
        return false;
    }
    if (status != Status::Ok) {
        return true;
    }

    info.protocolVersion = response[0];
    info.applicationAddress = readU32(&response[1]);
    info.flashEndAddress = readU32(&response[5]);
    info.maxChunkBytes = readU16(&response[9]);
    info.applicationValid = response[11] != 0;
    return true;
}

bool RipuSTM32WLBootloaderClient::beginUpdate(uint32_t imageSize, uint32_t imageCrc, Status &status)
{
    uint8_t request[8] = {};
    writeU32(&request[0], imageSize);
    writeU32(&request[4], imageCrc);
    size_t responseLength = 0;
    return command(Opcode::BootBegin, request, sizeof(request), nullptr, 0, responseLength, status) &&
           (status != Status::Ok || responseLength == 0);
}

bool RipuSTM32WLBootloaderClient::writeChunk(uint32_t offset, const uint8_t *data, size_t length, uint32_t &expectedOffset,
                                             Status &status)
{
    if (!data || length == 0 || length > kBootloaderMaxChunkBytes) {
        return false;
    }

    uint8_t request[4 + kBootloaderMaxChunkBytes] = {};
    writeU32(request, offset);
    std::memcpy(&request[4], data, length);
    uint8_t response[4] = {};
    size_t responseLength = 0;
    if (!command(Opcode::BootWrite, request, length + 4, response, sizeof(response), responseLength, status)) {
        return false;
    }
    if (status == Status::Ok && responseLength != sizeof(response)) {
        return false;
    }
    if (responseLength == sizeof(response)) {
        expectedOffset = readU32(response);
    }
    return true;
}

bool RipuSTM32WLBootloaderClient::verify(uint32_t &actualCrc, Status &status)
{
    uint8_t response[4] = {};
    size_t responseLength = 0;
    if (!command(Opcode::BootVerify, nullptr, 0, response, sizeof(response), responseLength, status)) {
        return false;
    }
    if (status == Status::Ok && responseLength != sizeof(response)) {
        return false;
    }
    if (responseLength == sizeof(response)) {
        actualCrc = readU32(response);
    }
    return true;
}

bool RipuSTM32WLBootloaderClient::runApplication(Status &status)
{
    size_t responseLength = 0;
    return command(Opcode::BootRun, nullptr, 0, nullptr, 0, responseLength, status) &&
           (status != Status::Ok || responseLength == 0);
}

bool RipuSTM32WLBootloaderClient::command(Opcode opcode, const uint8_t *request, size_t requestLength, uint8_t *response,
                                          size_t responseCapacity, size_t &responseLength, Status &status)
{
    responseLength = 0;
    status = Status::BadFrame;
    if (requestLength > kMaxPayloadBytes || !waitReady(kReadyTimeoutMs)) {
        return false;
    }

    buildFrame(opcode, request, requestLength);
    transfer(requestFrame_, nullptr, sizeof(requestFrame_));
    delay(1);
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (!waitReady(kReadyTimeoutMs)) {
            return false;
        }
        std::memset(responseFrame_, 0, sizeof(responseFrame_));
        transfer(nullptr, responseFrame_, sizeof(responseFrame_));
        if (readFrame(opcode, response, responseCapacity, responseLength, status)) {
            return true;
        }
        delay(1);
    }
    return false;
}

void RipuSTM32WLBootloaderClient::buildFrame(Opcode opcode, const uint8_t *payload, size_t payloadLength)
{
    std::memset(requestFrame_, 0, sizeof(requestFrame_));
    requestFrame_[0] = kMagic0;
    requestFrame_[1] = kMagic1;
    requestFrame_[2] = kProtocolVersion;
    requestFrame_[3] = static_cast<uint8_t>(opcode);
    requestFrame_[4] = ++sequence_;
    writeU16(&requestFrame_[6], payloadLength);
    writeU16(&requestFrame_[8], crc16(requestFrame_, 8));
    if (payloadLength > 0) {
        std::memcpy(&requestFrame_[kHeaderBytes], payload, payloadLength);
    }
    writeU16(&requestFrame_[kHeaderBytes + payloadLength], crc16(&requestFrame_[kHeaderBytes], payloadLength));
}

bool RipuSTM32WLBootloaderClient::readFrame(Opcode opcode, uint8_t *payload, size_t capacity, size_t &payloadLength,
                                            Status &status)
{
    if (responseFrame_[0] != kMagic0 || responseFrame_[1] != kMagic1 || responseFrame_[2] != kProtocolVersion ||
        responseFrame_[3] != static_cast<uint8_t>(opcode) || responseFrame_[4] != sequence_ ||
        readU16(&responseFrame_[8]) != crc16(responseFrame_, 8)) {
        return false;
    }

    const size_t length = readU16(&responseFrame_[6]);
    if (length > kMaxPayloadBytes || kHeaderBytes + length + kCrcBytes > sizeof(responseFrame_) || length > capacity ||
        readU16(&responseFrame_[kHeaderBytes + length]) != crc16(&responseFrame_[kHeaderBytes], length)) {
        return false;
    }

    status = static_cast<Status>(responseFrame_[5]);
    if (payload && length > 0) {
        std::memcpy(payload, &responseFrame_[kHeaderBytes], length);
    }
    payloadLength = length;
    return true;
}

bool RipuSTM32WLBootloaderClient::waitReady(uint32_t timeoutMs) const
{
    const uint32_t started = millis();
    while (digitalRead(busyPin_) == HIGH) {
        if (!Throttle::isWithinTimespanMs(started, timeoutMs)) {
            return false;
        }
        delay(1);
    }
    return true;
}

void RipuSTM32WLBootloaderClient::transfer(const uint8_t *mosi, uint8_t *miso, size_t length)
{
    concurrency::LockGuard guard(spiLock);

    spi_.beginTransaction(spiSettings_);
    digitalWrite(csPin_, LOW);
    delayMicroseconds(20);
    for (size_t i = 0; i < length; ++i) {
        const uint8_t received = spi_.transfer(mosi ? mosi[i] : 0);
        if (miso) {
            miso[i] = received;
        }
    }
    delayMicroseconds(5);
    digitalWrite(csPin_, HIGH);
    spi_.endTransaction();
}

void RipuSTM32WLBootloaderClient::writeU16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
}

void RipuSTM32WLBootloaderClient::writeU32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
    buffer[2] = static_cast<uint8_t>(value >> 16U);
    buffer[3] = static_cast<uint8_t>(value >> 24U);
}

uint16_t RipuSTM32WLBootloaderClient::readU16(const uint8_t *buffer)
{
    return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8U);
}

uint32_t RipuSTM32WLBootloaderClient::readU32(const uint8_t *buffer)
{
    return static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8U) |
           (static_cast<uint32_t>(buffer[2]) << 16U) | (static_cast<uint32_t>(buffer[3]) << 24U);
}

#endif
