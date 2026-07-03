#pragma once

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
#include "RipuSTM32WLTransport.h"
#include "concurrency/OSThread.h"

class RipuSTM32WLRadioInterface : public RadioInterface, protected concurrency::OSThread
{
  public:
    RipuSTM32WLRadioInterface(SPIClass &spi, SPISettings spiSettings, int csPin, int irqPin, int busyPin, int resetPin);

    bool init() override;
    bool reconfigure() override;
    bool canSleep() override;
    bool sleep() override;
    ErrorCode send(meshtastic_MeshPacket *p) override;
    meshtastic_QueueStatus getQueueStatus() override;
    bool cancelSending(NodeNum from, PacketId id) override;
    bool findInTxQueue(NodeNum from, PacketId id) override;
    void clampToLateRebroadcastWindow(NodeNum from, PacketId id) override;
    bool removePendingTXPacket(NodeNum from, PacketId id, uint32_t hopLimitLt) override;
    bool isIRQPending() override;

    uint32_t getPacketTime(uint32_t totalPacketLen, bool received = false) override;

  private:
    static constexpr uint32_t kPollIntervalMs = 20;
    static constexpr uint32_t kFallbackEventPollMs = 250;
    static constexpr uint32_t kTxWatchdogMs = 60000;
    static constexpr uint8_t kSyncWord = 0x2B;
    static constexpr int8_t kMinTxPowerDbm = -9;
    static constexpr int8_t kMaxTxPowerDbm = 22;

    int32_t runOnce() override;

    bool configureBridge();
    bool startReceive();
    bool wakeBridge();
    void logBridgeRadioError(const char *context);
    void stopReceiveState();
    void serviceEvents(bool force = false);
    void handleReceiveDone();
    void handleReceiveFailed();
    void handleTransmitDone(bool success);
    void processTransmitQueue();
    bool startTransmit(meshtastic_MeshPacket *txp);
    bool canSendImmediately();
    void setTransmitDelay();
    void setTransmitDelayFor(meshtastic_MeshPacket *p);
    void finishSending(bool success);
    void noteReceiveActivity(uint16_t events);
    void deliverRawPacket(const uint8_t *data, size_t length, const ripu_stm32wl::PacketMetadata &metadata);

    MeshPacketQueue txQueue_ = MeshPacketQueue(MAX_TX_QUEUE);
    RipuSTM32WLTransport transport_;
    bool bridgeReady_ = false;
    bool sleeping_ = false;
    bool receiving_ = false;
    bool receiveActive_ = false;
    uint32_t receiveActiveStartedMs_ = 0;
    uint32_t lastEventPollMs_ = 0;
    uint32_t txDelayStartedMs_ = 0;
    uint32_t txDelayMs_ = 0;
    bool txDelayActive_ = false;
    uint32_t rxGood_ = 0;
    uint32_t rxBad_ = 0;
    uint32_t txGood_ = 0;
    uint32_t txRelay_ = 0;
    uint16_t txDrop_ = 0;
};
