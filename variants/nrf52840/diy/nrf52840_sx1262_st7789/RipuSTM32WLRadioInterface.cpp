#include "configuration.h"

#if defined(USE_RIPU_STM32WL_RADIO)

#include "RipuSTM32WLRadioInterface.h"

#include "PowerMon.h"
#include "Throttle.h"
#include "airtime.h"
#include "error.h"
#include "main.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <cstring>

using namespace ripu_stm32wl;

RipuSTM32WLRadioInterface::RipuSTM32WLRadioInterface(SPIClass &spi, SPISettings spiSettings, int csPin, int irqPin,
                                                     int busyPin, int resetPin)
    : OSThread("RipuRadio", kPollIntervalMs), transport_(spi, spiSettings, csPin, irqPin, busyPin, resetPin)
{
}

bool RipuSTM32WLRadioInterface::init()
{
    RadioInterface::init();
    transport_.begin();

    bridgeReady_ = false;
    sleeping_ = false;
    if (!transport_.resetBridge()) {
        LOG_WARN("RIPU STM32WL bridge reset/ready failed");
        return false;
    }
    if (!transport_.hello()) {
        LOG_WARN("RIPU STM32WL bridge hello failed");
        return false;
    }
    if (!configureBridge()) {
        LOG_WARN("RIPU STM32WL bridge configure failed");
        return false;
    }
    if (!startReceive()) {
        return false;
    }

    bridgeReady_ = true;
    LOG_INFO("RIPU STM32WL bridge init success");
    return bridgeReady_;
}

bool RipuSTM32WLRadioInterface::reconfigure()
{
    RadioInterface::reconfigure();
    if (!bridgeReady_) {
        return false;
    }
    return configureBridge() && startReceive();
}

bool RipuSTM32WLRadioInterface::canSleep()
{
    const bool canSleepNow = txQueue_.empty() && sendingPacket == nullptr;
    if (!canSleepNow) {
        LOG_DEBUG("RIPU radio wait to sleep");
    }
    return canSleepNow;
}

bool RipuSTM32WLRadioInterface::sleep()
{
    if (!bridgeReady_) {
        return true;
    }
    receiving_ = false;
    receiveActive_ = false;
    txDelayActive_ = false;
    if (powerMon) {
        powerMon->clearState(meshtastic_PowerMon_State_Lora_RXOn);
        powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
    }
    const bool ok = transport_.sleep();
    if (ok) {
        sleeping_ = true;
    }
    return ok;
}

ErrorCode RipuSTM32WLRadioInterface::send(meshtastic_MeshPacket *p)
{
#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("send - lora tx disabled: Region unset");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }
#endif

    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("send - !config.lora.tx_enabled");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }

    if (p->to == NODENUM_BROADCAST_NO_LORA) {
        LOG_DEBUG("Drop no-LoRa pkt");
        return ERRNO_SHOULD_RELEASE;
    }

#ifndef LORA_DISABLE_SENDING
    if (sleeping_ && !startReceive()) {
        packetPool.release(p);
        return ERRNO_UNKNOWN;
    }

    printPacket("enqueue for send", p);
    LOG_DEBUG("ripu txGood=%u,txRelay=%u,rxGood=%u,rxBad=%u", txGood_, txRelay_, rxGood_, rxBad_);

    bool dropped = false;
    const ErrorCode res = txQueue_.enqueue(p, &dropped) ? ERRNO_OK : ERRNO_UNKNOWN;
    if (dropped) {
        txDrop_++;
    }
    if (res != ERRNO_OK) {
        packetPool.release(p);
        return res;
    }

    setTransmitDelay();
    return res;
#else
    packetPool.release(p);
    return ERRNO_DISABLED;
#endif
}

meshtastic_QueueStatus RipuSTM32WLRadioInterface::getQueueStatus()
{
    meshtastic_QueueStatus qs;
    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue_.getFree();
    qs.maxlen = txQueue_.getMaxLen();
    return qs;
}

bool RipuSTM32WLRadioInterface::cancelSending(NodeNum from, PacketId id)
{
    auto *p = txQueue_.remove(from, id);
    if (p) {
        packetPool.release(p);
    }

    const bool removed = p != nullptr;
    LOG_DEBUG("cancelSending id=0x%x, removed=%d", id, removed);
    return removed;
}

bool RipuSTM32WLRadioInterface::findInTxQueue(NodeNum from, PacketId id)
{
    return txQueue_.find(from, id);
}

void RipuSTM32WLRadioInterface::clampToLateRebroadcastWindow(NodeNum from, PacketId id)
{
    meshtastic_MeshPacket *p = txQueue_.remove(from, id, true, false);
    if (!p) {
        return;
    }

    p->tx_after = millis() + getTxDelayMsecWeightedWorst(p->rx_snr);
    bool dropped = false;
    if (txQueue_.enqueue(p, &dropped)) {
        LOG_DEBUG("Move existing queued packet to late rebroadcast window");
        setTransmitDelay();
    } else {
        packetPool.release(p);
    }
    if (dropped) {
        txDrop_++;
    }
}

bool RipuSTM32WLRadioInterface::removePendingTXPacket(NodeNum from, PacketId id, uint32_t hopLimitLt)
{
    meshtastic_MeshPacket *p = txQueue_.remove(from, id, true, true, hopLimitLt);
    if (!p) {
        return false;
    }

    LOG_DEBUG("Dropping pending-TX packet 0x%08x with hop limit %d", p->id, p->hop_limit);
    packetPool.release(p);
    setTransmitDelay();
    return true;
}

bool RipuSTM32WLRadioInterface::isIRQPending()
{
    return transport_.isIrqPending();
}

uint32_t RipuSTM32WLRadioInterface::getPacketTime(uint32_t totalPacketLen, bool received)
{
    (void)received;
    const float bandwidthHz = bw * 1000.0f;
    const bool headerDisabled = false;
    const float symbolTime = static_cast<float>(1U << sf) / bandwidthHz;
    const bool lowDataRateOptimize = symbolTime > 16e-3f;
    const float preambleTime = (preambleLength + 4.25f) * symbolTime;
    const float payloadSymbols =
        8.0f + std::max(ceilf(((8.0f * totalPacketLen - 4.0f * sf + 28.0f + 16.0f - 20.0f * headerDisabled) /
                               (4.0f * (sf - 2.0f * lowDataRateOptimize))) *
                              cr),
                         0.0f);
    return static_cast<uint32_t>((preambleTime + payloadSymbols * symbolTime) * 1000.0f);
}

int32_t RipuSTM32WLRadioInterface::runOnce()
{
    if (!bridgeReady_) {
        return 1000;
    }
    if (sleeping_) {
        return 1000;
    }

    serviceEvents();
    processTransmitQueue();
    return kPollIntervalMs;
}

bool RipuSTM32WLRadioInterface::configureBridge()
{
    if (!wakeBridge()) {
        return false;
    }

    limitPower(kMaxTxPowerDbm);
    if (power < kMinTxPowerDbm) {
        power = kMinTxPowerDbm;
    }

    RadioConfig bridgeConfig;
    bridgeConfig.frequencyHz = static_cast<uint32_t>(getFreq() * 1000000.0f + 0.5f);
    bridgeConfig.bandwidthKhzX10 = static_cast<uint16_t>(bw * 10.0f + 0.5f);
    bridgeConfig.spreadingFactor = sf;
    bridgeConfig.codingRate = cr;
    bridgeConfig.syncWord = kSyncWord;
    bridgeConfig.preambleSymbols = preambleLength;
    bridgeConfig.txPowerDbm = power;
    bridgeConfig.rxBoosted = 1;
    bridgeConfig.flags = ConfigExplicitHeader | ConfigCrcEnabled;
#if defined(RIPU_RADIO_TCXO_MILLIVOLTS)
    bridgeConfig.tcxoMillivolts = RIPU_RADIO_TCXO_MILLIVOLTS;
#endif

    LOG_INFO("RIPU bridge radio freq=%uHz bw=%.1fkHz sf=%u cr=4/%u pwr=%d tcxo=%umV", bridgeConfig.frequencyHz, bw, sf, cr,
             power, bridgeConfig.tcxoMillivolts);
    const bool ok = transport_.configure(bridgeConfig);
    if (!ok) {
        logBridgeRadioError("configure");
    }
    return ok;
}

bool RipuSTM32WLRadioInterface::startReceive()
{
    if (!wakeBridge()) {
        return false;
    }

    if (!transport_.startRx()) {
        LOG_WARN("RIPU bridge startRx failed");
        logBridgeRadioError("startRx");
        return false;
    }

    receiving_ = true;
    receiveActive_ = false;
    if (powerMon) {
        powerMon->setState(meshtastic_PowerMon_State_Lora_RXOn);
        powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
    }
    return true;
}

bool RipuSTM32WLRadioInterface::wakeBridge()
{
    if (!sleeping_) {
        return true;
    }

    if (!transport_.wake()) {
        LOG_WARN("RIPU bridge wake failed");
        return false;
    }

    sleeping_ = false;
    return true;
}

void RipuSTM32WLRadioInterface::logBridgeRadioError(const char *context)
{
    EventStatus event;
    if (transport_.getEvent(event) && (event.events & EventRadioError)) {
        LOG_WARN("RIPU bridge %s RadioLib error=%u", context, event.errorCode);
    }
}

void RipuSTM32WLRadioInterface::stopReceiveState()
{
    receiving_ = false;
    receiveActive_ = false;
    if (powerMon) {
        powerMon->clearState(meshtastic_PowerMon_State_Lora_RXOn);
    }
}

void RipuSTM32WLRadioInterface::serviceEvents(bool force)
{
    const uint32_t now = millis();
    const bool periodicPollDue = !Throttle::isWithinTimespanMs(lastEventPollMs_, kFallbackEventPollMs);
    if (!force && !transport_.isIrqPending() && !periodicPollDue) {
        return;
    }
    lastEventPollMs_ = now;

    EventStatus event;
    if (!transport_.getEvent(event)) {
        return;
    }

    noteReceiveActivity(event.events);

    if (event.events & EventRxDone) {
        handleReceiveDone();
    } else if (event.events & EventRxFailed) {
        handleReceiveFailed();
    }

    if (event.events & EventTxDone) {
        handleTransmitDone(true);
    } else if (event.events & EventTxFailed) {
        handleTransmitDone(false);
    }

    if (event.events & EventRadioError) {
        LOG_WARN("RIPU bridge radio error=0x%04x", event.errorCode);
        configureBridge();
        startReceive();
    }
}

void RipuSTM32WLRadioInterface::handleReceiveDone()
{
    uint8_t raw[MAX_LORA_PAYLOAD_LEN] = {};
    size_t length = 0;
    PacketMetadata metadata;
    stopReceiveState();

    if (!transport_.readPacket(raw, sizeof(raw), length, metadata)) {
        LOG_WARN("RIPU bridge readPacket failed");
        rxBad_++;
    } else {
        deliverRawPacket(raw, length, metadata);
    }

    startReceive();
    setTransmitDelay();
}

void RipuSTM32WLRadioInterface::handleReceiveFailed()
{
    stopReceiveState();
    rxBad_++;
    startReceive();
    setTransmitDelay();
}

void RipuSTM32WLRadioInterface::handleTransmitDone(bool success)
{
    if (powerMon) {
        powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
    }
    finishSending(success);
    startReceive();
    setTransmitDelay();
}

void RipuSTM32WLRadioInterface::processTransmitQueue()
{
    if (sendingPacket != nullptr) {
        if (!Throttle::isWithinTimespanMs(lastTxStart, kTxWatchdogMs)) {
            LOG_ERROR("Hardware Failure! RIPU bridge busyTx for more than 60s");
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_TRANSMIT_FAILED);
            rebootAtMsec = lastTxStart + kTxWatchdogMs + 5000;
        }
        return;
    }

    if (txQueue_.empty()) {
        txDelayActive_ = false;
        return;
    }

    if (!canSendImmediately()) {
        setTransmitDelay();
        return;
    }

    if (txDelayActive_ && Throttle::isWithinTimespanMs(txDelayStartedMs_, txDelayMs_)) {
        return;
    }
    txDelayActive_ = false;

    bool channelActive = false;
    if (!transport_.cad(channelActive)) {
        LOG_WARN("RIPU bridge CAD failed");
        startReceive();
        setTransmitDelay();
        return;
    }

    if (channelActive) {
        LOG_DEBUG("RIPU bridge channel active");
        startReceive();
        setTransmitDelay();
        return;
    }

    meshtastic_MeshPacket *txp = txQueue_.dequeue();
    assert(txp);
    startTransmit(txp);
    LOG_DEBUG("%d packets remain in RIPU TX queue", txQueue_.getMaxLen() - txQueue_.getFree());
}

bool RipuSTM32WLRadioInterface::startTransmit(meshtastic_MeshPacket *txp)
{
    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("Drop Tx packet because LoRa Tx disabled");
        packetPool.release(txp);
        return false;
    }
    if (!wakeBridge()) {
        packetPool.release(txp);
        return false;
    }

    stopReceiveState();
    if (powerMon) {
        powerMon->setState(meshtastic_PowerMon_State_Lora_TXOn);
    }

    const size_t numBytes = beginSending(txp);
    if (!transport_.transmit(reinterpret_cast<uint8_t *>(&radioBuffer), numBytes)) {
        LOG_ERROR("RIPU bridge transmit failed");
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_RADIO_SPI_BUG);
        finishSending(false);
        if (powerMon) {
            powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
        }
        startReceive();
        return false;
    }

    lastTxStart = millis();
    printPacket("Started Tx", txp);
    return true;
}

bool RipuSTM32WLRadioInterface::canSendImmediately()
{
    if (sendingPacket != nullptr) {
        LOG_WARN("Can not send yet, busyTx");
        return false;
    }

    if (receiving_ && receiveActive_) {
        if (Throttle::isWithinTimespanMs(receiveActiveStartedMs_, 2 * preambleTimeMsec)) {
            LOG_WARN("Can not send yet, busyRx");
            return false;
        }
        receiveActive_ = false;
    }

    return true;
}

void RipuSTM32WLRadioInterface::setTransmitDelay()
{
    setTransmitDelayFor(txQueue_.getFront());
}

void RipuSTM32WLRadioInterface::setTransmitDelayFor(meshtastic_MeshPacket *p)
{
    if (!p) {
        txDelayActive_ = false;
        return;
    }

    uint32_t delay = 1;
    const uint32_t now = millis();
    if (p->tx_after && static_cast<int32_t>(p->tx_after - now) > 0) {
        delay = p->tx_after - now;
    } else if (p->rx_snr == 0 && p->rx_rssi == 0) {
        delay = getTxDelayMsec();
    } else {
        LOG_DEBUG("rx_snr found. hop_limit:%d rx_snr:%f", p->hop_limit, p->rx_snr);
        delay = getTxDelayMsecWeighted(p);
    }

    txDelayStartedMs_ = now;
    txDelayMs_ = delay;
    txDelayActive_ = true;
}

void RipuSTM32WLRadioInterface::finishSending(bool success)
{
    auto *p = sendingPacket;
    sendingPacket = nullptr;
    if (!p) {
        return;
    }

    if (success) {
        const uint32_t xmitMsec = RadioInterface::getPacketTime(p);
        if (airTime) {
            airTime->logAirtime(TX_LOG, xmitMsec);
        }
        txGood_++;
        if (!isFromUs(p)) {
            txRelay_++;
        }
        printPacket("Completed sending", p);
    } else {
        txDrop_++;
        LOG_WARN("RIPU bridge dropped sending packet 0x%08x", p->id);
    }

    packetPool.release(p);
}

void RipuSTM32WLRadioInterface::noteReceiveActivity(uint16_t events)
{
    if (events & (EventPreambleDetected | EventHeaderValid)) {
        if (!receiveActive_) {
            receiveActiveStartedMs_ = millis();
        }
        receiveActive_ = true;
    }
    if (events & (EventRxDone | EventRxFailed)) {
        receiveActive_ = false;
    }
}

void RipuSTM32WLRadioInterface::deliverRawPacket(const uint8_t *data, size_t length, const PacketMetadata &metadata)
{
    const uint32_t rxMsec = getPacketTime(length, true);

#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("lora rx disabled: Region unset");
        if (airTime) {
            airTime->logAirtime(RX_ALL_LOG, rxMsec);
        }
        return;
    }
#endif

    if (length < sizeof(PacketHeader) || length > sizeof(radioBuffer)) {
        LOG_WARN("Ignore RIPU received packet length=%u", static_cast<unsigned>(length));
        rxBad_++;
        if (airTime) {
            airTime->logAirtime(RX_ALL_LOG, rxMsec);
        }
        return;
    }

    std::memcpy(&radioBuffer, data, length);
    const int32_t payloadLen = length - sizeof(PacketHeader);
    if (payloadLen < 0) {
        rxBad_++;
        return;
    }

    if (radioBuffer.header.from == 0) {
        LOG_WARN("Ignore received packet without sender");
        rxBad_++;
        return;
    }

    meshtastic_MeshPacket *mp = packetPool.allocZeroed();
    mp->from = radioBuffer.header.from;
    mp->to = radioBuffer.header.to;
    mp->id = radioBuffer.header.id;
    mp->channel = radioBuffer.header.channel;
    assert(HOP_MAX <= PACKET_FLAGS_HOP_LIMIT_MASK);
    mp->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
    mp->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
    mp->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
    mp->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
    mp->next_hop = mp->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : radioBuffer.header.next_hop;
    mp->relay_node = mp->hop_start == 0 ? NO_RELAY_NODE : radioBuffer.header.relay_node;
    mp->rx_snr = metadata.snrDbX4 / 4.0f;
    mp->rx_rssi = metadata.rssiDbmX4 / 4;
    mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    assert(static_cast<uint32_t>(payloadLen) <= sizeof(mp->encrypted.bytes));
    std::memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
    mp->encrypted.size = payloadLen;

    rxGood_++;
    printPacket("Lora RX", mp);
    if (airTime) {
        airTime->logAirtime(RX_LOG, rxMsec);
    }
    deliverToReceiver(mp);
}

#endif // USE_RIPU_STM32WL_RADIO
