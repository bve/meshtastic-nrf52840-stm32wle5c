#include "variant.h"

#include "Arduino.h"
#include "RipuFirmwareUpdater.h"
#include "mesh/NodeDB.h"

uint16_t getVDDVoltage();

static constexpr uint32_t DISPLAY_TIMEOUT_SECS = 60;

static void applyVariantConfigOverrides()
{
    config.display.screen_on_secs = DISPLAY_TIMEOUT_SECS;
    config.bluetooth.mode = meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN;
}

const uint32_t g_ADigitalPinMap[] = {
    // P0
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,

    // P1
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

static bool variantPinsInitialized = false;

static void initVariantPins()
{
    if (variantPinsInitialized) {
        return;
    }
    variantPinsInitialized = true;

#if defined(PIN_3V3_EN) && (PIN_3V3_EN >= 0)
    pinMode(PIN_3V3_EN, OUTPUT);
    digitalWrite(PIN_3V3_EN, HIGH);
#endif

#if defined(RIPULORA_NRST) && (RIPULORA_NRST >= 0)
    pinMode(RIPULORA_NRST, OUTPUT);
    digitalWrite(RIPULORA_NRST, HIGH);
#endif

#if defined(RIPU_RADIO_SPI_NSS) && (RIPU_RADIO_SPI_NSS >= 0)
    pinMode(RIPU_RADIO_SPI_NSS, OUTPUT);
    digitalWrite(RIPU_RADIO_SPI_NSS, HIGH);
#endif

#if defined(ADC_CTRL) && (ADC_CTRL >= 0)
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, !ADC_CTRL_ENABLED);
#endif

#if defined(ST7789_VDD_EN) && (ST7789_VDD_EN >= 0)
    pinMode(ST7789_VDD_EN, OUTPUT);
    digitalWrite(ST7789_VDD_EN, HIGH);
#endif

#if defined(PIN_BUTTON1) && (PIN_BUTTON1 >= 0)
    pinMode(PIN_BUTTON1, INPUT_PULLUP);
#endif
#if defined(ALT_BUTTON_PIN) && (ALT_BUTTON_PIN >= 0)
    pinMode(ALT_BUTTON_PIN, INPUT_PULLUP);
#endif

#if defined(VTFT_LEDA) && (VTFT_LEDA >= 0)
    pinMode(VTFT_LEDA, OUTPUT);
    digitalWrite(VTFT_LEDA, !TFT_BACKLIGHT_ON);
#endif

#if defined(ST7789_RESET) && (ST7789_RESET >= 0)
    pinMode(ST7789_RESET, OUTPUT);
    digitalWrite(ST7789_RESET, HIGH);
#endif

#if defined(PIN_LED1) && (PIN_LED1 >= 0)
    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, !LED_STATE_ON);
#endif
}

void earlyInitVariant()
{
    initVariantPins();
}

void initVariant()
{
    initVariantPins();
}

void lateInitVariant()
{
    applyVariantConfigOverrides();
    ripuFirmwareUsbInitialize();
}

void variantDefaultConfig()
{
    applyVariantConfigOverrides();
    config.lora.region = meshtastic_Config_LoRaConfig_RegionCode_KZ_433;
    config.lora.tx_power = 22;
}
