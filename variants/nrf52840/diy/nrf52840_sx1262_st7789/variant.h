#ifndef _VARIANT_PRIVATE_NRF52840_STM32WL_ST7789_
#define _VARIANT_PRIVATE_NRF52840_STM32WL_ST7789_

#define VARIANT_MCK (64000000ul)

#define USE_LFXO
#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// SuperMini switched 3.3V rail.
#define PIN_3V3_EN (0 + 13)

// LEDs / buttons
// This board has no firmware-controllable status LED. P0.15 is the onboard LED on common ProMicro/SuperMini boards.
#define PIN_LED1 (-1)
#define LED_STATE_ON 0
#define PIN_BUTTON1 (0 + 6)   // SW1
#define ALT_BUTTON_PIN (0 + 8) // SW2
#define BUTTON_ACTIVE_LOW 1
#define BUTTON_ACTIVE_PULLUP 1
#define ALT_BUTTON_ACTIVE_LOW 1
#define ALT_BUTTON_ACTIVE_PULLUP 1

// Battery sense: BAT+ -> 10k -> P0.31 -> 33k -> P0.29.
#define BATTERY_PIN (0 + 31)
#define ADC_CTRL (0 + 29)
#define ADC_CTRL_ENABLED LOW
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.6
#define ADC_MULTIPLIER (43.0F / 33.0F)

// Serial
#define PIN_SERIAL1_RX (-1)
#define PIN_SERIAL1_TX (-1)
#define PIN_SERIAL2_RX (-1)
#define PIN_SERIAL2_TX (-1)

// I2C
// Keep the I2C code path compiled because Meshtastic initializes SPI displays from it.
#define HAS_WIRE 0
#define I2C_NO_RESCAN
#define MESHTASTIC_EXCLUDE_ACCELEROMETER 1
#define MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR 1
#define MESHTASTIC_EXCLUDE_POWER_TELEMETRY 1
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA (-1)
#define PIN_WIRE_SCL (-1)

// SPI buses
#define SPI_INTERFACES_COUNT 2
#define PIN_SPI_MISO (32 + 2) // P1.02 -> L_MISO
#define PIN_SPI_MOSI (32 + 7) // P1.07 -> L_MOSI
#define PIN_SPI_SCK (32 + 1)  // P1.01 -> L_SCK

#define PIN_SPI1_MISO (0 + 12) // Dummy MISO; nRF52 SPI needs a valid pin.
#define PIN_SPI1_MOSI (0 + 22) // ST7789 SDA / MOSI
#define PIN_SPI1_SCK (0 + 17)  // ST7789 DC / SCLK

#ifndef NRF52840_STM32WL_ST7789_NO_RADIO
#define USE_RIPU_STM32WL_RADIO
#define RIPULORA_NRST (32 + 6) // P1.06 -> RipuLora NRST
#define RIPU_RADIO_SPI_NSS (32 + 11) // P1.11 -> L_CS / RipuLora PB2
#define LORA_SCK PIN_SPI_SCK
#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_CS RIPU_RADIO_SPI_NSS

#define RIPU_RADIO_IRQ (0 + 9)   // P0.09 -> bridge IRQ
#define RIPU_RADIO_BUSY (0 + 10) // P0.10 -> bridge BUSY

#define RIPU_RADIO_RESET RIPULORA_NRST // P1.06 -> RipuLora NRST
#endif

// ST7789 240x240 display over SPI via FPC.
#define HAS_SCREEN 1
#define HAS_SPI_TFT 1
#define ST7789_VDD_EN (0 + 24) // P0.24 -> display VDD
#define ST7789_CS_GROUNDED 1
#define ST7789_NSS (0 + 25) // Valid dummy CS; panel CS is tied to GND on this PCB.
#define ST7789_RS (0 + 11)     // WR
#define ST7789_SDA (0 + 22) // SDA / MOSI
#define ST7789_SCK (0 + 17) // DC / SCLK
#define ST7789_RESET (32 + 0)
#define ST7789_MISO PIN_SPI1_MISO
#define ST7789_BUSY (-1)
#define VTFT_LEDA (32 + 4) // P1.04 -> QLEDA gate
#define ST7789_BL VTFT_LEDA
#define TFT_BACKLIGHT_ON HIGH
#define ST7789_SPI_FREQUENCY 32000000U
#define ST7789_SPI_MODE SPI_MODE3
#define BRIGHTNESS_DEFAULT 255
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define TFT_OFFSET_X 0
#define TFT_OFFSET_Y 0

#define USE_ST7789

#ifdef __cplusplus
}
#endif

#endif
