# TeensyESP32C3Programmer

Arduino project for Teensy 4.1 that programs ESP32 targets from files on the Teensy SD card through either the native Espressif USB CDC bootloader or an external USB-serial adapter connected to the Teensy USB host port.

The project is built from `info/esp-serial-flasher` and keeps the sketch code allocation-free: no `malloc`, `new`, or Arduino `String`. Command lines, manifest lines, and flash chunks use fixed static buffers.

This copy uses the ROM bootloader path. SPI, SDIO, and flasher stub sources are intentionally removed. Native USB CDC mode is kept for ESP32-C3/S3/C5/H2/C6/P4/C61; USB-serial adapter mode uses UART protocol and can also program older ESP targets that do not expose USB CDC.

## Hardware

- Teensy 4.1.
- Powered USB hub connected to the Teensy USB host port.
- ESP32 target connected to the powered hub, either directly as native USB CDC or through a USB-serial adapter such as CH340/CH341, CP210x, FTDI, or PL2303.
- SD card mounted on the Teensy 4.1 built-in SD slot.

For ESP32-C3 native USB, wire ESP `EN` and `BOOT` to Teensy GPIO pins and set `ESP_EN_PIN` and `ESP_BOOT_PIN` at the top of `TeensyESP32C3Programmer.ino`. CDC DTR/RTS line-state changes are kept as a fallback, but many native USB boards do not use them to reset the chip or strap BOOT.

For ESP modules connected through CH340/CP210x/FTDI/PL2303, the adapter must expose `TX`, `RX`, `DTR`, and `RTS` to the ESP auto-reset circuit, or you must wire `ESP_EN_PIN` and `ESP_BOOT_PIN` to Teensy GPIOs. The sketch automatically switches to UART protocol when the connected USB serial device is not the Espressif native USB bootloader VID/PID.

## SD Files

For a normal Arduino/ESP32 image set, put these files in the SD root:

- `/bootloader.bin` at `0x0000`
- `/partitions.bin` at `0x8000`
- optional `/boot_app0.bin` at `0xE000`
- `/firmware.bin` at `0x10000`

Then run:

```text
flashset
```

For custom layouts, use `/flash.txt`:

```text
0x0000 /bootloader.bin
0x8000 /partitions.bin
0xE000 /boot_app0.bin
0x10000 /firmware.bin
```

Then run:

```text
flashman
```

You can also flash one file directly:

```text
flash 0x10000 /firmware.bin
```

## Serial Commands

Open the Teensy serial monitor at `115200`.

```text
help
ls
reset
boot
info
erase
flash <offset> <file>
flashapp <file>
flashman [file]
flashset
pass
```

`pass` opens a transparent serial bridge between the PC and the selected ESP USB serial port. Send `+++` to leave passthrough mode.

## Notes

- Flash writes are padded to 4-byte alignment with `0xFF`.
- The copied Espressif library files are under `src/` so Arduino IDE 2.x and Teensyduino compile them as part of the sketch.
- `ESP_SERIAL_FLASHER_LICENSE.txt` is the Apache 2.0 license copied from the source framework.
