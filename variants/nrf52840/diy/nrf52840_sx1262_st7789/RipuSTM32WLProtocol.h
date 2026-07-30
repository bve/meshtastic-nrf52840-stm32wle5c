#pragma once

#include <cstddef>
#include <cstdint>

namespace ripu_stm32wl {

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kMagic0 = 'R';
constexpr uint8_t kMagic1 = 'P';
constexpr size_t kMaxPacketBytes = 255;
constexpr size_t kMaxPayloadBytes = 270;
constexpr size_t kHeaderBytes = 10;
constexpr size_t kCrcBytes = 2;
constexpr size_t kMaxFrameBytes = kHeaderBytes + kMaxPayloadBytes + kCrcBytes;
constexpr size_t kRadioConfigLegacyPayloadBytes = 14;
constexpr size_t kRadioConfigPayloadBytes = 16;

enum class Opcode : uint8_t {
    Hello = 0x01,
    ResetRadio = 0x02,
    ConfigRadio = 0x03,
    StartRx = 0x04,
    StopRx = 0x05,
    TxPacket = 0x06,
    GetEvent = 0x07,
    ReadPacket = 0x08,
    Cad = 0x0A,
    Sleep = 0x0B,
    Wake = 0x0C,
    GetRandom = 0x0D,
};

enum class Status : uint8_t {
    Ok = 0x00,
    Busy = 0x01,
    BadFrame = 0x02,
    BadCommand = 0x03,
    RadioError = 0x04,
    Overflow = 0x05,
    NotReady = 0x06,
};

enum Event : uint16_t {
    EventTxDone = 0x0001,
    EventTxFailed = 0x0002,
    EventRxDone = 0x0004,
    EventRxFailed = 0x0008,
    EventPreambleDetected = 0x0010,
    EventHeaderValid = 0x0020,
    EventCadDone = 0x0040,
    EventCadDetected = 0x0080,
    EventConfigChanged = 0x0100,
    EventRadioError = 0x0200,
};

enum ConfigFlags : uint8_t {
    ConfigExplicitHeader = 0x01,
    ConfigCrcEnabled = 0x02,
    ConfigInvertIq = 0x04,
};

struct RadioConfig {
    uint32_t frequencyHz = 0;
    uint16_t bandwidthKhzX10 = 0;
    uint8_t spreadingFactor = 0;
    uint8_t codingRate = 0;
    uint8_t syncWord = 0x2B;
    uint16_t preambleSymbols = 16;
    int8_t txPowerDbm = 0;
    uint8_t rxBoosted = 1;
    uint8_t flags = ConfigExplicitHeader | ConfigCrcEnabled;
    uint16_t tcxoMillivolts = 0;
};

struct EventStatus {
    uint16_t events = 0;
    uint8_t rxLength = 0;
    int16_t rssiDbmX4 = 0;
    int16_t snrDbX4 = 0;
    uint16_t errorCode = 0;
};

struct PacketMetadata {
    int16_t rssiDbmX4 = 0;
    int16_t snrDbX4 = 0;
    int32_t frequencyErrorHz = 0;
};

uint16_t crc16(const uint8_t *data, size_t length);

} // namespace ripu_stm32wl
