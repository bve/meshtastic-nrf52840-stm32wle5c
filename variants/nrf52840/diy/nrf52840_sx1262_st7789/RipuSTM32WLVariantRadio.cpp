#include "configuration.h"

#if defined(USE_RIPU_STM32WL_RADIO)

#include "RadioLibInterface.h"
#include "RipuSTM32WLRadioInterface.h"
#include "detect/LoRaRadioType.h"
#include "main.h"

#include <memory>

#ifndef LORA_SPI_FREQUENCY
#define LORA_SPI_FREQUENCY 4000000
#endif

#ifndef RIPU_RADIO_SPI_FREQUENCY
#define RIPU_RADIO_SPI_FREQUENCY LORA_SPI_FREQUENCY
#endif

std::unique_ptr<RadioInterface> createVariantRadioInterface(LockingArduinoHal *hal, const SPISettings &spiSettings,
                                                            LoRaRadioType &detectedRadioType)
{
    (void)hal;
    (void)spiSettings;

    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24) {
        return nullptr;
    }

    SPISettings ripuSpiSettings(RIPU_RADIO_SPI_FREQUENCY, MSBFIRST, SPI_MODE0);
    detectedRadioType = STM32WLx_RADIO;

#if defined(HW_SPI1_DEVICE)
    return std::unique_ptr<RadioInterface>(new RipuSTM32WLRadioInterface(SPI1, ripuSpiSettings, RIPU_RADIO_SPI_NSS,
                                                                         RIPU_RADIO_IRQ, RIPU_RADIO_BUSY, RIPU_RADIO_RESET));
#else
    return std::unique_ptr<RadioInterface>(new RipuSTM32WLRadioInterface(SPI, ripuSpiSettings, RIPU_RADIO_SPI_NSS,
                                                                         RIPU_RADIO_IRQ, RIPU_RADIO_BUSY, RIPU_RADIO_RESET));
#endif
}

#endif // USE_RIPU_STM32WL_RADIO
