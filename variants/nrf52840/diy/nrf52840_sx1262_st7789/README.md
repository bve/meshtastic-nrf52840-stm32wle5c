# RIPULORA USB update

This variant exposes two nRF52840 USB CDC interfaces. The first remains the
Meshtastic serial API; the second is `RIPULORA DFU` and forwards firmware to the
STM32WLE5 SPI bootloader.

The STM32 bootloader and relocated bridge application are built in the
`STM32WLE5_RADIOLIB_BRIDGE` project:

```sh
pio run -e ripu_bootloader
pio run -e ripu_bridge
```

Install both STM32 images once with ST-Link, then install this Meshtastic
variant on the nRF52840. Future RIPULORA bridge updates need only USB:

```sh
python -m pip install pyserial
python variants/nrf52840/diy/nrf52840_sx1262_st7789/ripulora_flash.py \
  /path/to/STM32WLE5_RADIOLIB_BRIDGE/.pio/build/ripu_bridge/firmware.bin
```

The tool auto-detects the updater port. Use `--port /dev/ttyACM1` when several
matching devices are connected. If an update is interrupted, run the same
command again; the nRF52840 keeps RIPULORA in bootloader mode for recovery.
