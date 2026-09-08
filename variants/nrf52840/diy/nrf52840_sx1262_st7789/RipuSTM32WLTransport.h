#pragma once

#include "RipuSTM32WLProtocol.h"

#include <Arduino.h>
#include <SPI.h>
#include <cstddef>
#include <cstdint>

class RipuSTM32WLTransport
{
  public:
    RipuSTM32WLTransport(SPIClass &spi, SPISettings spiSettings, int csPin, int irqPin, int busyPin, int resetPin);

    void begin();
    bool resetBridge();
    bool hello();
    bool configure(const ripu_stm32wl::RadioConfig &config);
    bool startRx();
    bool stopRx();
    bool transmit(const uint8_t *data, size_t length);
    bool cad(bool &detected);
    bool getEvent(ripu_stm32wl::EventStatus &event);
    bool readPacket(uint8_t *buffer, size_t capacity, size_t &length, ripu_stm32wl::PacketMetadata &metadata);
    bool sleep();
    bool wake();
    bool randomBytes(uint8_t *buffer, size_t length);

    bool isIrqPending() const;
    bool isBusy() const;

  private:
    static constexpr uint32_t kReadyTimeoutMs = 250;
    static constexpr uint32_t kRadioCommandTimeoutMs = 1500;

    bool command(ripu_stm32wl::Opcode opcode, const uint8_t *request, size_t requestLength, uint8_t *response,
                 size_t responseCapacity, size_t &responseLength, uint32_t timeoutMs = kReadyTimeoutMs);
    bool writeFrame(ripu_stm32wl::Opcode opcode, const uint8_t *payload, size_t payloadLength);
    bool readFrame(ripu_stm32wl::Opcode opcode, uint8_t *payload, size_t capacity, size_t &payloadLength,
                   ripu_stm32wl::Status &status);
    void logBadFrame(ripu_stm32wl::Opcode opcode, const uint8_t *miso, const char *reason) const;
    bool waitReady(uint32_t timeoutMs) const;
    void transferUnlocked(const uint8_t *mosi, uint8_t *miso, size_t length);

    static void writeU16(uint8_t *buffer, uint16_t value);
    static void writeU32(uint8_t *buffer, uint32_t value);
    static uint16_t readU16(const uint8_t *buffer);
    static int16_t readI16(const uint8_t *buffer);
    static int32_t readI32(const uint8_t *buffer);

    SPIClass &spi_;
    SPISettings spiSettings_;
    int csPin_;
    int irqPin_;
    int busyPin_;
    int resetPin_;
    uint8_t sequence_ = 0;
};
