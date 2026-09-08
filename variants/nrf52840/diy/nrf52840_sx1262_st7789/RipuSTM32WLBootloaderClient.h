#pragma once

#include "RipuSTM32WLProtocol.h"

#include <Arduino.h>
#include <SPI.h>
#include <cstddef>
#include <cstdint>

struct RipuBootloaderInfo {
    uint8_t protocolVersion = 0;
    uint32_t applicationAddress = 0;
    uint32_t flashEndAddress = 0;
    uint16_t maxChunkBytes = 0;
    bool applicationValid = false;
};

class RipuSTM32WLBootloaderClient
{
  public:
    RipuSTM32WLBootloaderClient(SPIClass &spi, SPISettings spiSettings, int csPin, int busyPin, int resetPin);

    bool enter();
    void resetApplication();
    bool getInfo(RipuBootloaderInfo &info, ripu_stm32wl::Status &status);
    bool beginUpdate(uint32_t imageSize, uint32_t imageCrc, ripu_stm32wl::Status &status);
    bool writeChunk(uint32_t offset, const uint8_t *data, size_t length, uint32_t &expectedOffset, ripu_stm32wl::Status &status);
    bool verify(uint32_t &actualCrc, ripu_stm32wl::Status &status);
    bool runApplication(ripu_stm32wl::Status &status);

  private:
    static constexpr uint32_t kReadyTimeoutMs = 5000;

    bool command(ripu_stm32wl::Opcode opcode, const uint8_t *request, size_t requestLength, uint8_t *response,
                 size_t responseCapacity, size_t &responseLength, ripu_stm32wl::Status &status);
    void buildFrame(ripu_stm32wl::Opcode opcode, const uint8_t *payload, size_t payloadLength);
    bool readFrame(ripu_stm32wl::Opcode opcode, uint8_t *payload, size_t capacity, size_t &payloadLength,
                   ripu_stm32wl::Status &status);
    bool waitReady(uint32_t timeoutMs) const;
    void transfer(const uint8_t *mosi, uint8_t *miso, size_t length);

    static void writeU16(uint8_t *buffer, uint16_t value);
    static void writeU32(uint8_t *buffer, uint32_t value);
    static uint16_t readU16(const uint8_t *buffer);
    static uint32_t readU32(const uint8_t *buffer);

    SPIClass &spi_;
    SPISettings spiSettings_;
    int csPin_;
    int busyPin_;
    int resetPin_;
    uint8_t sequence_ = 0;
    uint8_t requestFrame_[ripu_stm32wl::kMaxFrameBytes] = {};
    uint8_t responseFrame_[ripu_stm32wl::kMaxFrameBytes] = {};
};
