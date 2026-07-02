#include "configuration.h"

#if defined(USE_RIPU_STM32WL_RADIO)

#include "RipuSTM32WLTransport.h"

#include "main.h"
#include "SPILock.h"
#include "Throttle.h"

#include <cstring>

using namespace ripu_stm32wl;

RipuSTM32WLTransport::RipuSTM32WLTransport(SPIClass &spi, SPISettings spiSettings, int csPin, int irqPin, int busyPin,
                                           int resetPin)
    : spi_(spi), spiSettings_(spiSettings), csPin_(csPin), irqPin_(irqPin), busyPin_(busyPin), resetPin_(resetPin)
{
}

void RipuSTM32WLTransport::begin()
{
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);

    if (irqPin_ >= 0) {
        pinMode(irqPin_, INPUT);
    }
    if (busyPin_ >= 0) {
        pinMode(busyPin_, INPUT);
    }
    if (resetPin_ >= 0) {
        pinMode(resetPin_, OUTPUT);
        digitalWrite(resetPin_, HIGH);
    }

    spi_.begin();
}

bool RipuSTM32WLTransport::resetBridge()
{
    if (resetPin_ < 0) {
        return true;
    }

    digitalWrite(resetPin_, LOW);
    delay(10);
    digitalWrite(resetPin_, HIGH);
    delay(100);
    return waitReady(kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::hello()
{
    uint8_t response[4] = {};
    size_t responseLength = 0;
    const bool ok = command(Opcode::Hello, nullptr, 0, response, sizeof(response), responseLength);
    if (!ok || responseLength < 1 || response[0] != kProtocolVersion) {
        LOG_WARN("RIPU hello ok=%u len=%u data=%02X %02X", ok, static_cast<unsigned>(responseLength), response[0], response[1]);
        return false;
    }
    return true;
}

bool RipuSTM32WLTransport::configure(const RadioConfig &config)
{
    uint8_t payload[14] = {};
    writeU32(&payload[0], config.frequencyHz);
    writeU16(&payload[4], config.bandwidthKhzX10);
    payload[6] = config.spreadingFactor;
    payload[7] = config.codingRate;
    payload[8] = config.syncWord;
    writeU16(&payload[9], config.preambleSymbols);
    payload[11] = static_cast<uint8_t>(config.txPowerDbm);
    payload[12] = config.rxBoosted;
    payload[13] = config.flags;

    size_t responseLength = 0;
    return command(Opcode::ConfigRadio, payload, sizeof(payload), nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::startRx()
{
    size_t responseLength = 0;
    return command(Opcode::StartRx, nullptr, 0, nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::stopRx()
{
    size_t responseLength = 0;
    return command(Opcode::StopRx, nullptr, 0, nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::transmit(const uint8_t *data, size_t length)
{
    size_t responseLength = 0;
    return command(Opcode::TxPacket, data, length, nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::cad(bool &detected)
{
    uint8_t response[1] = {};
    size_t responseLength = 0;
    if (!command(Opcode::Cad, nullptr, 0, response, sizeof(response), responseLength, kRadioCommandTimeoutMs) || responseLength < 1) {
        return false;
    }
    detected = response[0] != 0;
    return true;
}

bool RipuSTM32WLTransport::getEvent(EventStatus &event)
{
    uint8_t response[9] = {};
    size_t responseLength = 0;
    if (!command(Opcode::GetEvent, nullptr, 0, response, sizeof(response), responseLength) || responseLength < sizeof(response)) {
        return false;
    }

    event.events = readU16(&response[0]);
    event.rxLength = response[2];
    event.rssiDbmX4 = readI16(&response[3]);
    event.snrDbX4 = readI16(&response[5]);
    event.errorCode = readU16(&response[7]);
    return true;
}

bool RipuSTM32WLTransport::readPacket(uint8_t *buffer, size_t capacity, size_t &length, PacketMetadata &metadata)
{
    uint8_t response[kMaxPayloadBytes] = {};
    size_t responseLength = 0;
    if (!command(Opcode::ReadPacket, nullptr, 0, response, sizeof(response), responseLength, kRadioCommandTimeoutMs) ||
        responseLength < 8) {
        return false;
    }

    const size_t packetLength = responseLength - 8;
    if (packetLength > capacity) {
        return false;
    }

    metadata.rssiDbmX4 = readI16(&response[0]);
    metadata.snrDbX4 = readI16(&response[2]);
    metadata.frequencyErrorHz = readI32(&response[4]);
    std::memcpy(buffer, &response[8], packetLength);
    length = packetLength;
    return true;
}

bool RipuSTM32WLTransport::sleep()
{
    size_t responseLength = 0;
    return command(Opcode::Sleep, nullptr, 0, nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::wake()
{
    size_t responseLength = 0;
    return command(Opcode::Wake, nullptr, 0, nullptr, 0, responseLength, kRadioCommandTimeoutMs);
}

bool RipuSTM32WLTransport::randomBytes(uint8_t *buffer, size_t length)
{
    if (!buffer || length > kMaxPayloadBytes) {
        return false;
    }

    uint8_t request[1] = {static_cast<uint8_t>(length)};
    size_t responseLength = 0;
    if (!command(Opcode::GetRandom, request, sizeof(request), buffer, length, responseLength, kRadioCommandTimeoutMs)) {
        return false;
    }
    return responseLength == length;
}

bool RipuSTM32WLTransport::isIrqPending() const
{
    return irqPin_ >= 0 && digitalRead(irqPin_) == HIGH;
}

bool RipuSTM32WLTransport::isBusy() const
{
    return busyPin_ >= 0 && digitalRead(busyPin_) == HIGH;
}

bool RipuSTM32WLTransport::command(Opcode opcode, const uint8_t *request, size_t requestLength, uint8_t *response,
                                   size_t responseCapacity, size_t &responseLength, uint32_t timeoutMs)
{
    responseLength = 0;
    if (requestLength > kMaxPayloadBytes || !waitReady(timeoutMs) || !writeFrame(opcode, request, requestLength)) {
        return false;
    }

    delayMicroseconds(50);
    Status status = Status::BadFrame;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (!waitReady(timeoutMs)) {
            LOG_WARN("RIPU RPC op=%02X seq=%u ready timeout before read", static_cast<uint8_t>(opcode), sequence_);
            return false;
        }
        if (readFrame(opcode, response, responseCapacity, responseLength, status)) {
            if (status != Status::Ok) {
                LOG_WARN("RIPU RPC op=%02X seq=%u status=%u", static_cast<uint8_t>(opcode), sequence_,
                         static_cast<uint8_t>(status));
            }
            return status == Status::Ok;
        }
        delay(1);
    }
    LOG_WARN("RIPU RPC op=%02X seq=%u read failed", static_cast<uint8_t>(opcode), sequence_);
    return false;
}

bool RipuSTM32WLTransport::writeFrame(Opcode opcode, const uint8_t *payload, size_t payloadLength)
{
    uint8_t frame[kMaxFrameBytes] = {};
    const uint8_t seq = ++sequence_;
    frame[0] = kMagic0;
    frame[1] = kMagic1;
    frame[2] = kProtocolVersion;
    frame[3] = static_cast<uint8_t>(opcode);
    frame[4] = seq;
    frame[5] = 0;
    writeU16(&frame[6], payloadLength);
    writeU16(&frame[8], crc16(frame, 8));
    if (payloadLength > 0) {
        std::memcpy(&frame[kHeaderBytes], payload, payloadLength);
    }
    writeU16(&frame[kHeaderBytes + payloadLength], crc16(&frame[kHeaderBytes], payloadLength));

    transfer(frame, nullptr, sizeof(frame));
    return true;
}

bool RipuSTM32WLTransport::readFrame(Opcode opcode, uint8_t *payload, size_t capacity, size_t &payloadLength, Status &status)
{
    uint8_t mosi[kMaxFrameBytes] = {};
    uint8_t miso[kMaxFrameBytes] = {};
    transfer(mosi, miso, sizeof(miso));

    if (miso[0] != kMagic0 || miso[1] != kMagic1 || miso[2] != kProtocolVersion ||
        miso[3] != static_cast<uint8_t>(opcode) || miso[4] != sequence_) {
        logBadFrame(opcode, miso, "hdr");
        return false;
    }
    if (readU16(&miso[8]) != crc16(miso, 8)) {
        logBadFrame(opcode, miso, "hcrc");
        return false;
    }

    const size_t length = readU16(&miso[6]);
    if (length > kMaxPayloadBytes || length + kHeaderBytes + kCrcBytes > sizeof(miso)) {
        logBadFrame(opcode, miso, "len");
        return false;
    }
    if (readU16(&miso[kHeaderBytes + length]) != crc16(&miso[kHeaderBytes], length)) {
        logBadFrame(opcode, miso, "pcrc");
        return false;
    }

    status = static_cast<Status>(miso[5]);
    if (length > capacity) {
        return false;
    }
    if (length > 0 && payload != nullptr) {
        std::memcpy(payload, &miso[kHeaderBytes], length);
    }
    payloadLength = length;
    return true;
}

void RipuSTM32WLTransport::logBadFrame(Opcode opcode, const uint8_t *miso, const char *reason) const
{
    LOG_WARN("RIPU RPC bad %s op=%02X seq=%u miso=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             reason, static_cast<uint8_t>(opcode), sequence_, miso[0], miso[1], miso[2], miso[3], miso[4], miso[5],
             miso[6], miso[7], miso[8], miso[9], miso[10], miso[11]);
}

bool RipuSTM32WLTransport::waitReady(uint32_t timeoutMs) const
{
    if (busyPin_ < 0) {
        return true;
    }

    const uint32_t started = millis();
    while (digitalRead(busyPin_) == HIGH) {
        if (!Throttle::isWithinTimespanMs(started, timeoutMs)) {
            return false;
        }
        delay(1);
    }
    return true;
}

void RipuSTM32WLTransport::transfer(const uint8_t *mosi, uint8_t *miso, size_t length)
{
    concurrency::LockGuard guard(spiLock);

    spi_.beginTransaction(spiSettings_);
    digitalWrite(csPin_, LOW);
    delayMicroseconds(20);
    for (size_t i = 0; i < length; ++i) {
        const uint8_t out = mosi != nullptr ? mosi[i] : 0;
        const uint8_t in = spi_.transfer(out);
        if (miso != nullptr) {
            miso[i] = in;
        }
    }
    delayMicroseconds(5);
    digitalWrite(csPin_, HIGH);
    spi_.endTransaction();
}

void RipuSTM32WLTransport::writeU16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = static_cast<uint8_t>(value & 0xFFU);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
}

void RipuSTM32WLTransport::writeU32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = static_cast<uint8_t>(value & 0xFFU);
    buffer[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    buffer[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    buffer[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

uint16_t RipuSTM32WLTransport::readU16(const uint8_t *buffer)
{
    return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8U);
}

int16_t RipuSTM32WLTransport::readI16(const uint8_t *buffer)
{
    return static_cast<int16_t>(readU16(buffer));
}

int32_t RipuSTM32WLTransport::readI32(const uint8_t *buffer)
{
    return static_cast<int32_t>(static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8U) |
                                (static_cast<uint32_t>(buffer[2]) << 16U) | (static_cast<uint32_t>(buffer[3]) << 24U));
}

#endif // USE_RIPU_STM32WL_RADIO
