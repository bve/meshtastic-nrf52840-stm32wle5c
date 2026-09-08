#include "configuration.h"

#if defined(USE_RIPU_STM32WL_RADIO)

#include "RipuFirmwareUpdater.h"

#include "RipuSTM32WLBootloaderClient.h"
#include "RipuSTM32WLProtocol.h"
#include "RipuSTM32WLRadioInterface.h"
#include "concurrency/OSThread.h"
#include "variant.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <SPI.h>
#include <atomic>
#include <cstring>

namespace
{

constexpr uint8_t kUsbMagic[] = {'R', 'P', 'D', 'F'};
constexpr uint8_t kUsbProtocolVersion = 1;
constexpr size_t kUsbHeaderBytes = 16;
constexpr size_t kUsbMaxPayloadBytes = ripu_stm32wl::kBootloaderMaxChunkBytes;

enum class UsbOpcode : uint8_t {
    Probe = 0x01,
    Begin = 0x02,
    Data = 0x03,
    Verify = 0x04,
    Run = 0x05,
    Abort = 0x06,
    Health = 0x07,
};

enum class UsbStatus : uint32_t {
    Ok = 0,
    BadFrame = 1,
    BadCommand = 2,
    TransportError = 3,
    TargetRejected = 4,
    IncompatibleTarget = 5,
    InvalidState = 6,
};

std::atomic<bool> updateActive{false};
std::atomic<bool> recoveryRequested{false};

uint16_t readU16(const uint8_t *buffer)
{
    return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8U);
}

uint32_t readU32(const uint8_t *buffer)
{
    return static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8U) |
           (static_cast<uint32_t>(buffer[2]) << 16U) | (static_cast<uint32_t>(buffer[3]) << 24U);
}

void writeU16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
}

void writeU32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
    buffer[2] = static_cast<uint8_t>(value >> 16U);
    buffer[3] = static_cast<uint8_t>(value >> 24U);
}

class RipuFirmwareUpdater : private concurrency::OSThread
{
  public:
    RipuFirmwareUpdater()
        : concurrency::OSThread("RipuUpdater", 5), client_(SPI, SPISettings(RIPU_RADIO_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
                                                           RIPU_RADIO_SPI_NSS, RIPU_RADIO_BUSY, RIPULORA_NRST)
    {
    }

    void initialize()
    {
        usb_.begin(115200);
        if (TinyUSBDevice.mounted()) {
            TinyUSBDevice.detach();
            delay(10);
            TinyUSBDevice.attach();
        }
    }

  private:
    int32_t runOnce() override
    {
        poll();
        return 5;
    }

    void poll()
    {
        const bool connected = usb_.dtr() != 0;
        if (!connected) {
            if (connected_) {
                resetParser();
            }
            connected_ = false;
            return;
        }
        if (!connected_) {
            resetParser();
            connected_ = true;
        }

        while (usb_.available() > 0) {
            const int value = usb_.read();
            if (value >= 0) {
                consume(static_cast<uint8_t>(value));
            }
        }
    }

    void consume(uint8_t value)
    {
        if (headerLength_ < kUsbHeaderBytes) {
            header_[headerLength_++] = value;
            if (headerLength_ <= sizeof(kUsbMagic) && std::memcmp(header_, kUsbMagic, headerLength_) != 0) {
                headerLength_ = value == kUsbMagic[0] ? 1 : 0;
                if (headerLength_ == 1) {
                    header_[0] = value;
                }
                return;
            }
            if (headerLength_ < kUsbHeaderBytes) {
                return;
            }

            payloadExpected_ = readU16(&header_[6]);
            if (header_[4] != kUsbProtocolVersion || payloadExpected_ > sizeof(payload_)) {
                sendResponse(header_[5], UsbStatus::BadFrame, nullptr, 0);
                resetParser();
                return;
            }
            if (payloadExpected_ == 0) {
                processFrame();
            }
            return;
        }

        payload_[payloadLength_++] = value;
        if (payloadLength_ == payloadExpected_) {
            processFrame();
        }
    }

    void processFrame()
    {
        const uint8_t opcode = header_[5];
        if (readU32(&header_[12]) != ripu_stm32wl::crc32(payload_, payloadLength_)) {
            sendResponse(opcode, UsbStatus::BadFrame, nullptr, 0);
            resetParser();
            return;
        }

        switch (static_cast<UsbOpcode>(opcode)) {
        case UsbOpcode::Probe:
            handleProbe(opcode);
            break;
        case UsbOpcode::Begin:
            handleBegin(opcode);
            break;
        case UsbOpcode::Data:
            handleData(opcode);
            break;
        case UsbOpcode::Verify:
            handleVerify(opcode);
            break;
        case UsbOpcode::Run:
            handleRun(opcode);
            break;
        case UsbOpcode::Abort:
            handleAbort(opcode);
            break;
        case UsbOpcode::Health:
            handleHealth(opcode);
            break;
        default:
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            break;
        }
        resetParser();
    }

    void handleProbe(uint8_t opcode)
    {
        if (payloadLength_ != 0) {
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            return;
        }

        uint8_t response[8] = {};
        response[0] = kUsbProtocolVersion;
        writeU16(&response[1], kUsbMaxPayloadBytes);
        response[3] = updateActive.load(std::memory_order_acquire) ? 1 : 0;
        writeU32(&response[4], ripu_stm32wl::kApplicationAddress);
        sendResponse(opcode, UsbStatus::Ok, response, sizeof(response));
    }

    void handleBegin(uint8_t opcode)
    {
        if (payloadLength_ != 8) {
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            return;
        }

        updateActive.store(true, std::memory_order_release);
        recoveryRequested.store(true, std::memory_order_release);
        sessionStarted_ = false;
        verified_ = false;
        if (!client_.enter()) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }

        RipuBootloaderInfo info;
        ripu_stm32wl::Status targetStatus = ripu_stm32wl::Status::BadFrame;
        if (!client_.getInfo(info, targetStatus)) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }
        if (targetStatus != ripu_stm32wl::Status::Ok) {
            sendTargetError(opcode, targetStatus);
            return;
        }
        if (info.protocolVersion != ripu_stm32wl::kBootloaderProtocolVersion ||
            info.applicationAddress != ripu_stm32wl::kApplicationAddress || info.flashEndAddress <= info.applicationAddress ||
            info.maxChunkBytes == 0 || info.maxChunkBytes > kUsbMaxPayloadBytes) {
            sendResponse(opcode, UsbStatus::IncompatibleTarget, nullptr, 0);
            return;
        }

        if (!client_.beginUpdate(readU32(payload_), readU32(&payload_[4]), targetStatus)) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }
        if (targetStatus != ripu_stm32wl::Status::Ok) {
            sendTargetError(opcode, targetStatus);
            return;
        }

        sessionStarted_ = true;
        maxChunkBytes_ = info.maxChunkBytes;
        uint8_t response[12] = {};
        response[0] = info.protocolVersion;
        writeU32(&response[1], info.applicationAddress);
        writeU32(&response[5], info.flashEndAddress);
        writeU16(&response[9], info.maxChunkBytes);
        response[11] = info.applicationValid ? 1 : 0;
        sendResponse(opcode, UsbStatus::Ok, response, sizeof(response));
    }

    void handleData(uint8_t opcode)
    {
        if (!sessionStarted_) {
            sendResponse(opcode, UsbStatus::InvalidState, nullptr, 0);
            return;
        }
        if (payloadLength_ == 0 || payloadLength_ > maxChunkBytes_) {
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            return;
        }

        uint32_t expectedOffset = 0;
        ripu_stm32wl::Status targetStatus = ripu_stm32wl::Status::BadFrame;
        if (!client_.writeChunk(readU32(&header_[8]), payload_, payloadLength_, expectedOffset, targetStatus)) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }
        if (targetStatus != ripu_stm32wl::Status::Ok) {
            sendTargetError(opcode, targetStatus);
            return;
        }

        uint8_t response[4] = {};
        writeU32(response, expectedOffset);
        sendResponse(opcode, UsbStatus::Ok, response, sizeof(response));
    }

    void handleVerify(uint8_t opcode)
    {
        if (!sessionStarted_ || payloadLength_ != 0) {
            sendResponse(opcode, UsbStatus::InvalidState, nullptr, 0);
            return;
        }

        uint32_t actualCrc = 0;
        ripu_stm32wl::Status targetStatus = ripu_stm32wl::Status::BadFrame;
        if (!client_.verify(actualCrc, targetStatus)) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }
        if (targetStatus != ripu_stm32wl::Status::Ok) {
            sendTargetError(opcode, targetStatus);
            return;
        }

        verified_ = true;
        uint8_t response[4] = {};
        writeU32(response, actualCrc);
        sendResponse(opcode, UsbStatus::Ok, response, sizeof(response));
    }

    void handleRun(uint8_t opcode)
    {
        if (!sessionStarted_ || !verified_ || payloadLength_ != 0) {
            sendResponse(opcode, UsbStatus::InvalidState, nullptr, 0);
            return;
        }

        ripu_stm32wl::Status targetStatus = ripu_stm32wl::Status::BadFrame;
        if (!client_.runApplication(targetStatus)) {
            sendResponse(opcode, UsbStatus::TransportError, nullptr, 0);
            return;
        }
        if (targetStatus != ripu_stm32wl::Status::Ok) {
            sendTargetError(opcode, targetStatus);
            return;
        }
        sendResponse(opcode, UsbStatus::Ok, nullptr, 0);
        sessionStarted_ = false;
        verified_ = false;
        updateActive.store(false, std::memory_order_release);
    }

    void handleAbort(uint8_t opcode)
    {
        if (payloadLength_ != 0) {
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            return;
        }

        updateActive.store(true, std::memory_order_release);
        recoveryRequested.store(true, std::memory_order_release);
        client_.resetApplication();
        sessionStarted_ = false;
        verified_ = false;
        updateActive.store(false, std::memory_order_release);
        sendResponse(opcode, UsbStatus::Ok, nullptr, 0);
    }

    void handleHealth(uint8_t opcode)
    {
        if (payloadLength_ != 0) {
            sendResponse(opcode, UsbStatus::BadCommand, nullptr, 0);
            return;
        }

        const uint8_t health = static_cast<uint8_t>(ripuSTM32WLBridgeHealth());
        sendResponse(opcode, UsbStatus::Ok, &health, sizeof(health));
    }

    void sendTargetError(uint8_t opcode, ripu_stm32wl::Status targetStatus)
    {
        const uint8_t response = static_cast<uint8_t>(targetStatus);
        sendResponse(opcode, UsbStatus::TargetRejected, &response, sizeof(response));
    }

    void sendResponse(uint8_t opcode, UsbStatus status, const uint8_t *payload, size_t payloadLength)
    {
        uint8_t header[kUsbHeaderBytes] = {};
        std::memcpy(header, kUsbMagic, sizeof(kUsbMagic));
        header[4] = kUsbProtocolVersion;
        header[5] = opcode | 0x80U;
        writeU16(&header[6], payloadLength);
        writeU32(&header[8], static_cast<uint32_t>(status));
        writeU32(&header[12], ripu_stm32wl::crc32(payload, payloadLength));
        usb_.write(header, sizeof(header));
        if (payload && payloadLength > 0) {
            usb_.write(payload, payloadLength);
        }
        usb_.flush();
    }

    void resetParser()
    {
        headerLength_ = 0;
        payloadLength_ = 0;
        payloadExpected_ = 0;
    }

    Adafruit_USBD_CDC usb_;
    RipuSTM32WLBootloaderClient client_;
    uint8_t header_[kUsbHeaderBytes] = {};
    uint8_t payload_[kUsbMaxPayloadBytes] = {};
    size_t headerLength_ = 0;
    size_t payloadLength_ = 0;
    size_t payloadExpected_ = 0;
    uint16_t maxChunkBytes_ = 0;
    bool sessionStarted_ = false;
    bool verified_ = false;
    bool connected_ = false;
};

RipuFirmwareUpdater &firmwareUpdater()
{
    static RipuFirmwareUpdater updater;
    return updater;
}

} // namespace

void ripuFirmwareUsbInitialize()
{
    firmwareUpdater().initialize();
}

bool ripuFirmwareUpdateActive()
{
    return updateActive.load(std::memory_order_acquire);
}

bool ripuFirmwareTakeRecoveryRequest()
{
    return recoveryRequested.exchange(false, std::memory_order_acq_rel);
}

#else

#include "RipuFirmwareUpdater.h"

void ripuFirmwareUsbInitialize() {}
bool ripuFirmwareUpdateActive()
{
    return false;
}
bool ripuFirmwareTakeRecoveryRequest()
{
    return false;
}

#endif
