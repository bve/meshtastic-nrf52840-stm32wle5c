// Variant-local ST7789 driver for the private nRF52840 board.
// It intentionally shadows the upstream meshtastic-st7789 header via this variant's include path.
#ifndef PRIVATE_ST7789SPI_H
#define PRIVATE_ST7789SPI_H

#include "OLEDDisplay.h"
#include <Arduino.h>
#include <SPI.h>
#include "nrfx_coredep.h"

struct TFTColorRegion {
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t height;
    uint16_t onColorBe;
    uint16_t offColorBe;
    bool enabled = false;
};

#define ST77XX_SWRESET 0x01
#define ST77XX_SLPOUT 0x11
#define ST77XX_NORON 0x13
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A
#define ST77XX_PORCTRL 0xB2
#define ST77XX_GCTRL 0xB7
#define ST77XX_VCOMS 0xBB
#define ST77XX_LCMCTRL 0xC0
#define ST77XX_VDVVRHEN 0xC2
#define ST77XX_VRHS 0xC3
#define ST77XX_VDVS 0xC4
#define ST77XX_FRCTRL2 0xC6
#define ST77XX_PWCTRL1 0xD0
#define ST77XX_PVGAMCTRL 0xE0
#define ST77XX_NVGAMCTRL 0xE1

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_RGB 0x00

#ifndef ST7789_SPI_FREQUENCY
#define ST7789_SPI_FREQUENCY 8000000U
#endif

#ifndef ST7789_SPI_MODE
#define ST7789_SPI_MODE SPI_MODE0
#endif

#ifndef TFT_OFFSET_X
#define TFT_OFFSET_X 0
#endif

#ifndef TFT_OFFSET_Y
#define TFT_OFFSET_Y 0
#endif

#ifndef BRIGHTNESS_DEFAULT
#define BRIGHTNESS_DEFAULT 128
#endif

static inline void st7789DelayMs(uint32_t ms)
{
    for (uint32_t elapsed = 0; elapsed < ms; elapsed++) {
        nrfx_coredep_delay_us(1000);
    }
}

class ST7789Spi : public OLEDDisplay {
  public:
    ST7789Spi(SPIClass *spiClass, uint8_t rst, uint8_t dc, uint8_t cs, OLEDDISPLAY_GEOMETRY g = GEOMETRY_RAWMODE,
              uint16_t width = 240, uint16_t height = 240, int mosi = -1, int miso = -1, int clk = -1)
        : _rst(rst), _dc(dc), _cs(cs), _miso(miso), _mosi(mosi), _clk(clk), _spi(spiClass),
          _spiSettings(ST7789_SPI_FREQUENCY, MSBFIRST, ST7789_SPI_MODE)
    {
        setGeometry(g, width, height);
        setRGB(0xFFFF);
    }

    bool connect() override
    {
        _buffheight = displayHeight / 8 + ((displayHeight % 8) ? 1 : 0);

#if defined(VTFT_CTRL) && (VTFT_CTRL >= 0)
        pinMode(VTFT_CTRL, OUTPUT);
        digitalWrite(VTFT_CTRL, LOW);
        st7789DelayMs(20);
#endif

        if (_cs != kNoPin) {
            pinMode(_cs, OUTPUT);
            digitalWrite(_cs, HIGH);
        }
        pinMode(_dc, OUTPUT);
        digitalWrite(_dc, HIGH);
        if (_rst != kNoPin) {
            pinMode(_rst, OUTPUT);
        }
        prepareIdleBus();

#ifdef ESP_PLATFORM
        _spi->begin(_clk, _miso, _mosi, -1);
#else
        _spi->begin();
#endif
        _spi->beginTransaction(_spiSettings);
        _spi->endTransaction();
        prepareIdleBus();

        if (_rst != kNoPin) {
            digitalWrite(_rst, HIGH);
            st7789DelayMs(5);
            digitalWrite(_rst, LOW);
            st7789DelayMs(20);
            digitalWrite(_rst, HIGH);
            st7789DelayMs(120);
        }
        _connected = true;
        return true;
    }

    void display(void) override
    {
        if (!ensureConnected()) {
            return;
        }
        ensureScanlineBuffer(displayWidth);
        if (_scanlineBuf == nullptr) {
            return;
        }
        applyBacklightBrightness();

        select();
        _spi->beginTransaction(_spiSettings);
        setAddrWindow(0, 0, displayWidth, displayHeight);

        for (uint16_t y = 0; y < displayHeight; y++) {
            for (uint16_t x = 0; x < displayWidth; x++) {
#if defined(ST7789_SOFTWARE_ROTATE_90)
                const uint16_t srcX = y;
                const uint16_t srcY = displayHeight - 1 - x;
#elif defined(ST7789_SOFTWARE_ROTATE_270)
                const uint16_t srcX = displayWidth - 1 - y;
                const uint16_t srcY = x;
#else
                const uint16_t srcX = x;
                const uint16_t srcY = y;
#endif
                _scanlineBuf[x] = resolvePixelColorBe(srcX, srcY, isPixelSet(srcX, srcY));
            }
            _spi->transfer(reinterpret_cast<void *>(_scanlineBuf), nullptr, static_cast<uint32_t>(2 * displayWidth));
        }

        _spi->endTransaction();
        deselect();
    }

    void resetOrientation()
    {
        setPanelOrientation(ST77XX_MADCTL_RGB);
    }

    void flipScreenVertically()
    {
        setPanelOrientation(ST77XX_MADCTL_RGB);
    }

    void mirrorScreen()
    {
        setPanelOrientation(ST77XX_MADCTL_RGB);
    }

    void displayOn(void)
    {
        sendCommand(ST77XX_DISPON);
        applyBacklightBrightness();
    }
    void displayOff(void)
    {
        setBacklightRaw(false);
        sendCommand(ST77XX_DISPOFF);
        _connected = false;
    }

    void setBrightness(uint8_t brightness) override
    {
        _brightness = brightness;
        applyBacklightBrightness();
    }

    void setRGB(uint16_t color, TFTColorRegion *colorRegions = nullptr)
    {
        _colorRegions = colorRegions;
        _onColorBe = swap16(color);
        _offColorBe = 0x0000;
    }

    ~ST7789Spi() override
    {
        if (_scanlineBuf != nullptr) {
            free(_scanlineBuf);
            _scanlineBuf = nullptr;
        }
    }

  protected:
    void sendInitCommands() override
    {
        if (!ensureConnected()) {
            return;
        }

        sendCommand(ST77XX_SWRESET);
        st7789DelayMs(150);
        sendCommand(ST77XX_SLPOUT);
        st7789DelayMs(150);

        writeCommandData(ST77XX_COLMOD, 0x55);
        writeCommandData(ST77XX_MADCTL, _madctl);

        const uint8_t porch[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        writeCommandData(ST77XX_PORCTRL, porch, sizeof(porch));

        writeCommandData(ST77XX_GCTRL, 0x35);
        writeCommandData(ST77XX_VCOMS, 0x1F);
        writeCommandData(ST77XX_LCMCTRL, 0x2C);
        writeCommandData(ST77XX_VDVVRHEN, 0x01);
        writeCommandData(ST77XX_VRHS, 0x12);
        writeCommandData(ST77XX_VDVS, 0x20);
        writeCommandData(ST77XX_FRCTRL2, 0x0F);

        const uint8_t power[] = {0xA4, 0xA1};
        writeCommandData(ST77XX_PWCTRL1, power, sizeof(power));

        const uint8_t gammaPositive[] = {0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39,
                                         0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D};
        writeCommandData(ST77XX_PVGAMCTRL, gammaPositive, sizeof(gammaPositive));

        const uint8_t gammaNegative[] = {0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39,
                                         0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31};
        writeCommandData(ST77XX_NVGAMCTRL, gammaNegative, sizeof(gammaNegative));

        sendCommand(ST77XX_NORON);
        st7789DelayMs(10);
        sendCommand(ST77XX_INVON);
        st7789DelayMs(10);
        sendCommand(ST77XX_DISPON);
        st7789DelayMs(100);
    }

  private:
    static constexpr uint8_t kNoPin = 0xFF;

    uint8_t _rst;
    uint8_t _dc;
    uint8_t _cs;
    int _miso;
    int _mosi;
    int _clk;
    SPIClass *_spi;
    SPISettings _spiSettings;
    uint8_t _buffheight = 0;
    bool _connected = false;
    uint8_t _madctl = ST77XX_MADCTL_RGB;
    uint16_t _onColorBe = 0xFFFF;
    uint16_t _offColorBe = 0x0000;
    uint16_t *_scanlineBuf = nullptr;
    uint16_t _scanlineBufCapacity = 0;
    uint8_t _brightness = BRIGHTNESS_DEFAULT;
    TFTColorRegion *_colorRegions = nullptr;

    void setGeometry(OLEDDISPLAY_GEOMETRY g, uint16_t width, uint16_t height)
    {
        geometry = g;
        displayWidth = width ? width : 128;
        displayHeight = height ? height : 64;
        maxDisplayHeight = ((displayHeight + 7) / 8) * 8;
        displayBufferSize = displayWidth * maxDisplayHeight / 8;
    }

    int getBufferOffset(void) override { return 0; }

    static uint16_t swap16(uint16_t value) { return static_cast<uint16_t>((value >> 8) | (value << 8)); }

    bool isPixelSet(uint16_t x, uint16_t y) const
    {
        const uint16_t row = y >> 3;
        const uint8_t bitMask = 1u << (y & 0x07);
        return (buffer[row * displayWidth + x] & bitMask) != 0;
    }

    uint16_t resolvePixelColorBe(uint16_t x, uint16_t y, bool pixelSet) const
    {
        const TFTColorRegion *region = findColorRegion(x, y);
        if (region != nullptr) {
            return pixelSet ? region->onColorBe : region->offColorBe;
        }
        return pixelSet ? _onColorBe : _offColorBe;
    }

    const TFTColorRegion *findColorRegion(uint16_t x, uint16_t y) const
    {
        static constexpr uint8_t kMaxColorRegions = 48;
        if (_colorRegions == nullptr) {
            return nullptr;
        }
        for (uint8_t i = 0; i < kMaxColorRegions && _colorRegions[i].enabled; i++) {
            const TFTColorRegion &region = _colorRegions[i];
            if (x >= region.x && y >= region.y && x < region.x + region.width && y < region.y + region.height) {
                return &region;
            }
        }
        return nullptr;
    }

    bool ensureConnected()
    {
        if (_connected) {
            return true;
        }
        return connect();
    }

    void setPanelOrientation(uint8_t madctl)
    {
        _madctl = madctl;
        writeCommandData(ST77XX_MADCTL, _madctl);
    }

    void applyBacklightBrightness()
    {
#if defined(ST7789_BL) && (ST7789_BL >= 0)
        pinMode(ST7789_BL, OUTPUT);
        const uint8_t pwm = TFT_BACKLIGHT_ON == LOW ? static_cast<uint8_t>(255 - _brightness) : _brightness;
        analogWrite(ST7789_BL, pwm);
#endif
    }

    void prepareIdleBus()
    {
        static constexpr bool kClockIdleHigh = ST7789_SPI_MODE == SPI_MODE2 || ST7789_SPI_MODE == SPI_MODE3;
#if defined(ST7789_SCK) && (ST7789_SCK >= 0)
        pinMode(ST7789_SCK, OUTPUT);
        digitalWrite(ST7789_SCK, kClockIdleHigh ? HIGH : LOW);
#endif
#if defined(ST7789_SDA) && (ST7789_SDA >= 0)
        pinMode(ST7789_SDA, OUTPUT);
        digitalWrite(ST7789_SDA, LOW);
#endif
        digitalWrite(_dc, HIGH);
    }

    void setBacklightRaw(bool on)
    {
#if defined(ST7789_BL) && (ST7789_BL >= 0)
        pinMode(ST7789_BL, OUTPUT);
        if (on) {
            applyBacklightBrightness();
        } else {
            analogWrite(ST7789_BL, TFT_BACKLIGHT_ON == LOW ? 255 : 0);
        }
#endif
    }

    void ensureScanlineBuffer(uint16_t width)
    {
        if (_scanlineBuf != nullptr && width <= _scanlineBufCapacity) {
            return;
        }
        if (_scanlineBuf != nullptr) {
            free(_scanlineBuf);
        }
        _scanlineBuf = static_cast<uint16_t *>(malloc(width * sizeof(uint16_t)));
        _scanlineBufCapacity = _scanlineBuf ? width : 0;
    }

    void select()
    {
        if (_cs != kNoPin) {
            digitalWrite(_cs, LOW);
        }
    }

    void deselect()
    {
        if (_cs != kNoPin) {
            digitalWrite(_cs, HIGH);
        }
    }

    void sendCommand(uint8_t command)
    {
        if (!ensureConnected()) {
            return;
        }
        deselect();
        digitalWrite(_dc, LOW);
        select();
        _spi->beginTransaction(_spiSettings);
        _spi->transfer(command);
        _spi->endTransaction();
        deselect();
        digitalWrite(_dc, HIGH);
    }

    void writeCommandData(uint8_t command, uint8_t data)
    {
        if (!ensureConnected()) {
            return;
        }
        deselect();
        digitalWrite(_dc, LOW);
        select();
        _spi->beginTransaction(_spiSettings);
        _spi->transfer(command);
        digitalWrite(_dc, HIGH);
        _spi->transfer(data);
        _spi->endTransaction();
        deselect();
    }

    void writeCommandData(uint8_t command, const uint8_t *data, size_t length)
    {
        if (!ensureConnected()) {
            return;
        }
        deselect();
        digitalWrite(_dc, LOW);
        select();
        _spi->beginTransaction(_spiSettings);
        _spi->transfer(command);
        digitalWrite(_dc, HIGH);
        if (length > 0) {
            _spi->transfer(const_cast<uint8_t *>(data), static_cast<uint32_t>(length));
        }
        _spi->endTransaction();
        deselect();
    }

    void writeCommandInTransaction(uint8_t command)
    {
        digitalWrite(_dc, LOW);
        _spi->transfer(command);
        digitalWrite(_dc, HIGH);
    }

    void write16(uint16_t value)
    {
        _spi->transfer(value >> 8);
        _spi->transfer(value & 0xFF);
    }

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
    {
        const uint16_t x0 = x + TFT_OFFSET_X;
        const uint16_t y0 = y + TFT_OFFSET_Y;
        const uint16_t x1 = x0 + w - 1;
        const uint16_t y1 = y0 + h - 1;

        writeCommandInTransaction(ST77XX_CASET);
        write16(x0);
        write16(x1);
        writeCommandInTransaction(ST77XX_RASET);
        write16(y0);
        write16(y1);
        writeCommandInTransaction(ST77XX_RAMWR);
    }
};

#endif
