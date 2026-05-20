/*
 * Teensy 4.1 ESP32-C3 programmer.
 *
 * Host: Teensy 4.1 with USBHost_t36 and SD card.
 * Target: ESP32 native USB CDC bootloader or USB-serial adapter.
 *
 * No malloc/new/String are used by this sketch. All command and flash buffers
 * are static so the project stays deterministic on real hardware.
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <USBHost_t36.h>
#include <string.h>

extern "C" {
#include "src/esp_loader.h"
#include "src/esp_loader_error.h"
#include "src/esp_loader_io.h"
}

#define EN_OLD_DTRRTS 0 // old not working

// Recommended for ESP32-C3 native USB: wire these Teensy GPIOs to ESP EN and BOOT.
// Leave as 255 to use USB CDC DTR/RTS only, but native USB boards often do not
// reset or enter ROM bootloader from CDC line-state changes alone.
static const uint8_t ESP_EN_PIN = 255;
static const uint8_t ESP_BOOT_PIN = 255;

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t ESP_UART_FLASH_BAUD = SERIAL_BAUD;
static const uint32_t FLASH_BLOCK_SIZE = 4096;
static const int SD_CS_PIN = BUILTIN_SDCARD;

static USBHost usbHost;
static USBHub usbHub1(usbHost);
static USBHub usbHub2(usbHost);
static USBSerial espSerial(usbHost);

typedef struct {
    esp_loader_port_t base;
    uint32_t deadline_ms;
} teensy_usb_port_t;

static teensy_usb_port_t teensyPort;
static esp_loader_t loader;
static bool loaderReady = false;
static bool sdReady = false;
static bool targetAlreadyInBootloader = false;
static bool espSerialActive = false;
static uint32_t espPortBaud = SERIAL_BAUD;

static char commandLine[128];
static uint8_t commandLen = 0;
static uint8_t flashBuffer[FLASH_BLOCK_SIZE];
static char manifestLine[128];

static esp_loader_error_t teensyPortInit(esp_loader_port_t *port);
static void teensyPortDeinit(esp_loader_port_t *port);
static void teensyEnterBootloader(esp_loader_port_t *port);
static void teensyResetTarget(esp_loader_port_t *port);
static void teensyEnterBootloaderPins();
static void teensyStartTimer(esp_loader_port_t *port, uint32_t ms);
static uint32_t teensyRemainingTime(esp_loader_port_t *port);
static void teensyDelayMs(esp_loader_port_t *port, uint32_t ms);
static void teensyDebugPrint(esp_loader_port_t *port, const char *str);
static esp_loader_error_t teensyChangeRate(esp_loader_port_t *port, uint32_t rate);
static esp_loader_error_t teensyWrite(esp_loader_port_t *port, const uint8_t *data, uint16_t size, uint32_t timeout);
static esp_loader_error_t teensyRead(esp_loader_port_t *port, uint8_t *data, uint16_t size, uint32_t timeout);

static const esp_loader_port_ops_t teensyOps = {
    teensyPortInit,
    teensyPortDeinit,
    teensyEnterBootloader,
    teensyResetTarget,
    teensyStartTimer,
    teensyRemainingTime,
    teensyDelayMs,
    teensyDebugPrint,
    teensyChangeRate,
    teensyWrite,
    teensyRead,
    NULL,
    NULL,
    NULL,
    NULL,
};

static void printHelp();
static void pollCommand();
static void pollEspSerial();
static void executeCommand(char *line);
static bool connectBootloader();
static bool waitForEspUsb(uint32_t timeoutMs);
static void clearEspInput();
static bool initLoaderForConnectedSerial();
static bool isNativeEspUsbBootloader();
static bool flashFileAtOffset(const char *path, uint32_t offset);
static bool flashManifest(const char *path);
static bool printTargetInfo();
static bool eraseChip();
static void listSdRoot();
static void passthroughMode();
static char *nextToken(char **cursor);
static bool commandEquals(const char *a, const char *b);
static bool parseU32(const char *text, uint32_t *value);
static const char *chipName(target_chip_t chip);
static const char *usbSerialTypeName(USBSerialBase::sertype_t type);
static const char *loaderErrorName(esp_loader_error_t err);
static uint32_t align4(uint32_t value);

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    usbHost.begin();

    if (ESP_EN_PIN != 255) {
        pinMode(ESP_EN_PIN, OUTPUT);
        digitalWrite(ESP_EN_PIN, HIGH);
    }
    if (ESP_BOOT_PIN != 255) {
        pinMode(ESP_BOOT_PIN, OUTPUT);
        digitalWrite(ESP_BOOT_PIN, HIGH);
    }

    sdReady = SD.begin(SD_CS_PIN);
    teensyPort.base.ops = &teensyOps;
    loaderReady = true;

    Serial.println();
    Serial.println("Teensy 4.1 ESP32-C3 Programmer");
    Serial.println(sdReady ? "SD: ready" : "SD: not found");
    Serial.println(loaderReady ? "esp-serial-flasher: ready" : "esp-serial-flasher: init failed");
    printHelp();
}

void loop()
{
    usbHost.Task();
    pollEspSerial();
    pollCommand();
}

static esp_loader_error_t teensyPortInit(esp_loader_port_t *port)
{
    (void)port;
    espSerial.begin(espPortBaud);
    return ESP_LOADER_SUCCESS;
}

static void teensyPortDeinit(esp_loader_port_t *port)
{
    (void)port;
    espSerial.end();
    espPortBaud = SERIAL_BAUD;
}

static void teensyEnterBootloader(esp_loader_port_t *port)
{
    (void)port;
    if (targetAlreadyInBootloader) {
        Serial.println("ESP: bootloader already requested");
        return;
    }
    teensyEnterBootloaderPins();
}

static void teensyEnterBootloaderPins()
{
    Serial.println("ESP: bootloader reset");

    if (ESP_BOOT_PIN != 255 && ESP_EN_PIN != 255) {
        Serial.println("ESP: using GPIO EN/BOOT");
        digitalWrite(ESP_BOOT_PIN, LOW);
        digitalWrite(ESP_EN_PIN, LOW);
        delay(100);
        digitalWrite(ESP_EN_PIN, HIGH);
        delay(250);
        digitalWrite(ESP_BOOT_PIN, HIGH);
        targetAlreadyInBootloader = true;
        return;
    }

    if (!waitForEspUsb(1000)) {
        Serial.println("ESP USB serial not connected before boot; waiting for USB host");
    }
    if (!waitForEspUsb(5000)) {
        Serial.println("ESP USB serial not connected");
        return;
    }
    espPortBaud = SERIAL_BAUD;
    espSerial.begin(espPortBaud);

    if (!isNativeEspUsbBootloader()) {
        Serial.println("ESP: using USB-serial DTR/RTS boot sequence");

#if EN_OLD_DTRRTS
		espSerial.setDTR(false);
		espSerial.setRTS(false);
        //espSerial.setControlLines(false, false);
        delay(100);
		espSerial.setDTR(false);
		espSerial.setRTS(true);
        //espSerial.setControlLines(false, true);
        delay(100);
		espSerial.setDTR(true);
		espSerial.setRTS(false);
        //espSerial.setControlLines(true, false);
        delay(500);
		espSerial.setDTR(false);
		espSerial.setRTS(false);
        //espSerial.setControlLines(false, false);
        delay(250);

#else
        espSerial.setControlLines(false, false);
        delay(100);
        espSerial.setControlLines(false, true);
        delay(100);
        espSerial.setControlLines(true, false);
        delay(500);
        espSerial.setControlLines(false, false);
        delay(250);
#endif

        targetAlreadyInBootloader = true;
        return;
    }

    Serial.println("ESP: using USB CDC DTR/RTS Arduino-style boot sequence");

#if EN_OLD_DTRRTS
	espSerial.setDTR(false);
	espSerial.setRTS(false);
    //espSerial.setControlLines(false, false);
    delay(100);
	espSerial.setDTR(true);
	espSerial.setRTS(false);
    //espSerial.setControlLines(true, false);
    delay(100);
	espSerial.setDTR(true);
	espSerial.setRTS(true);
    //espSerial.setControlLines(true, true);
    delay(10);
	espSerial.setDTR(false);
	espSerial.setRTS(true);
    //espSerial.setControlLines(false, true);
    delay(100);
	espSerial.setDTR(false);
	espSerial.setRTS(false);
    //espSerial.setControlLines(false, false);
#else

    espSerial.setControlLines(false, false);
    delay(100);
    espSerial.setControlLines(true, false);
    delay(100);
    espSerial.setControlLines(true, true);
    delay(10);
    espSerial.setControlLines(false, true);
    delay(100);
    espSerial.setControlLines(false, false);
#endif

    waitForEspUsb(3000);
    targetAlreadyInBootloader = true;
}

static void teensyResetTarget(esp_loader_port_t *port)
{
    (void)port;
    targetAlreadyInBootloader = false;
    Serial.println("ESP: normal reset");

    if (ESP_EN_PIN != 255) {
        Serial.println("ESP: using GPIO EN");
        if (ESP_BOOT_PIN != 255) {
            digitalWrite(ESP_BOOT_PIN, HIGH);
        }
        digitalWrite(ESP_EN_PIN, LOW);
        delay(100);
        digitalWrite(ESP_EN_PIN, HIGH);
        delay(250);
        return;
    }

    Serial.println("ESP: using CDC DTR/RTS Arduino-style reset sequence");

#if EN_OLD_DTRRTS

	espSerial.setDTR(false);
	espSerial.setRTS(true);

    //espSerial.setControlLines(false, true);
    delay(100);
    //espSerial.setControlLines(false, false);

	espSerial.setDTR(false);
	espSerial.setRTS(false);
	
#else
    espSerial.setControlLines(false, true);
    delay(100);
    espSerial.setControlLines(false, false);
#endif
    waitForEspUsb(3000);
}

static void teensyStartTimer(esp_loader_port_t *port, uint32_t ms)
{
    teensy_usb_port_t *p = container_of(port, teensy_usb_port_t, base);
    p->deadline_ms = millis() + ms;
}

static uint32_t teensyRemainingTime(esp_loader_port_t *port)
{
    teensy_usb_port_t *p = container_of(port, teensy_usb_port_t, base);
    const int32_t remaining = (int32_t)(p->deadline_ms - millis());
    return remaining > 0 ? (uint32_t)remaining : 0;
}

static void teensyDelayMs(esp_loader_port_t *port, uint32_t ms)
{
    (void)port;
    delay(ms);
}

static void teensyDebugPrint(esp_loader_port_t *port, const char *str)
{
    (void)port;
    Serial.println(str);
}

static esp_loader_error_t teensyChangeRate(esp_loader_port_t *port, uint32_t rate)
{
    (void)port;
    espSerial.end();
    delay(20);
    espSerial.begin(rate);
    espPortBaud = rate;
    return ESP_LOADER_SUCCESS;
}

static esp_loader_error_t teensyWrite(esp_loader_port_t *port, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)port;
    const uint32_t start = millis();
    uint16_t written = 0;

    while (written < size) {
        usbHost.Task();
        const int result = espSerial.write(data + written, size - written);
        if (result > 0) {
            written += (uint16_t)result;
        } else if ((millis() - start) >= timeout) {
            return ESP_LOADER_ERROR_TIMEOUT;
        } else {
            yield();
        }
    }

    return ESP_LOADER_SUCCESS;
}

static esp_loader_error_t teensyRead(esp_loader_port_t *port, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)port;
    const uint32_t start = millis();
    uint16_t count = 0;

    while (count < size) {
        usbHost.Task();
        if (espSerial.available()) {
            data[count++] = (uint8_t)espSerial.read();
        } else if ((millis() - start) >= timeout) {
            return ESP_LOADER_ERROR_TIMEOUT;
        } else {
            yield();
        }
    }

    return ESP_LOADER_SUCCESS;
}

static void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  ls");
    Serial.println("  reset");
    Serial.println("  boot");
    Serial.println("  info                   (USB CDC native or USB-serial adapters)");
    Serial.println("  erase");
    Serial.println("  flash <offset> <file>");
    Serial.println("  flashapp <file>        (offset 0x10000)");
    Serial.println("  flashman [file]        (default /flash.txt)");
    Serial.println("  flashset               (/bootloader.bin, /partitions.bin, /boot_app0.bin, /firmware.bin)");
    Serial.println("  pass                   (+++ exits)");
}

static void pollCommand()
{
    while (Serial.available()) {
        const char c = (char)Serial.read();

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            commandLine[commandLen] = '\0';
            if (commandLen > 0) {
                executeCommand(commandLine);
            }
            commandLen = 0;
            Serial.print("> ");
            continue;
        }
        if ((c == '\b' || c == 127) && commandLen > 0) {
            commandLen--;
            continue;
        }
        if (commandLen < sizeof(commandLine) - 1) {
            commandLine[commandLen++] = c;
        }
    }
}

static void pollEspSerial()
{
    const bool connected = (bool)espSerial;
    if (connected == espSerialActive) {
        return;
    }

    espSerialActive = connected;
    if (connected) {
        Serial.print("USB serial connected ");
        Serial.print(espSerial.idVendor(), HEX);
        Serial.print(":");
        Serial.print(espSerial.idProduct(), HEX);
        Serial.print(" type=");
        Serial.println(usbSerialTypeName(espSerial.serialType()));
        espPortBaud = SERIAL_BAUD;
        espSerial.begin(espPortBaud);
    } else {
        Serial.println("USB serial disconnected");
        targetAlreadyInBootloader = false;
        espPortBaud = SERIAL_BAUD;
    }
}

static void executeCommand(char *line)
{
    char *cursor = line;
    char *cmd = nextToken(&cursor);
    if (cmd == NULL) {
        return;
    }

    if (commandEquals(cmd, "help") || commandEquals(cmd, "?")) {
        printHelp();
    } else if (commandEquals(cmd, "ls")) {
        listSdRoot();
    } else if (commandEquals(cmd, "reset")) {
        teensyResetTarget(&teensyPort.base);
    } else if (commandEquals(cmd, "boot")) {
        targetAlreadyInBootloader = false;
        teensyEnterBootloaderPins();
    } else if (commandEquals(cmd, "info")) {
        printTargetInfo();
    } else if (commandEquals(cmd, "erase")) {
        eraseChip();
    } else if (commandEquals(cmd, "flash")) {
        uint32_t offset = 0;
        char *offsetText = nextToken(&cursor);
        char *fileName = nextToken(&cursor);
        if (offsetText == NULL || fileName == NULL || !parseU32(offsetText, &offset)) {
            Serial.println("Usage: flash <offset> <file>");
            return;
        }
        flashFileAtOffset(fileName, offset);
    } else if (commandEquals(cmd, "flashapp")) {
        char *fileName = nextToken(&cursor);
        if (fileName == NULL) {
            Serial.println("Usage: flashapp <file>");
            return;
        }
        flashFileAtOffset(fileName, 0x10000);
    } else if (commandEquals(cmd, "flashman")) {
        char *fileName = nextToken(&cursor);
        flashManifest(fileName != NULL ? fileName : "/flash.txt");
    } else if (commandEquals(cmd, "flashset")) {
        bool ok = true;
        ok = flashFileAtOffset("/bootloader.bin", 0x0000) && ok;
        ok = flashFileAtOffset("/partitions.bin", 0x8000) && ok;
        if (SD.exists("/boot_app0.bin")) {
            ok = flashFileAtOffset("/boot_app0.bin", 0xE000) && ok;
        }
        ok = flashFileAtOffset("/firmware.bin", 0x10000) && ok;
        Serial.println(ok ? "flashset: done" : "flashset: failed");
        if (ok) {
            teensyResetTarget(&teensyPort.base);
        }
    } else if (commandEquals(cmd, "pass")) {
        passthroughMode();
    } else {
        Serial.println("Unknown command. Type help.");
    }
}

static bool connectBootloader()
{
    if (!loaderReady) {
        Serial.println("Loader not ready");
        return false;
    }
    if (!waitForEspUsb(3000)) {
        Serial.println("ESP USB serial not connected");
        return false;
    }

    if (!initLoaderForConnectedSerial()) {
        return false;
    }

    Serial.print("USB serial VID:PID ");
    Serial.print(espSerial.idVendor(), HEX);
    Serial.print(":");
    Serial.print(espSerial.idProduct(), HEX);
    Serial.print(" type=");
    Serial.print(usbSerialTypeName(espSerial.serialType()));
    Serial.print(" protocol=");
    Serial.print(isNativeEspUsbBootloader() ? "USB" : "UART");
    if (isNativeEspUsbBootloader()) {
        Serial.print(" ctrl=");
        Serial.print(espSerial.cdcControlInterface());
        Serial.print(" data=");
        Serial.print(espSerial.cdcDataInterface());
    }
    Serial.println();

    clearEspInput();
    esp_loader_connect_args_t args;
    args.sync_timeout = 100;
    args.trials = 10;
    const esp_loader_error_t err = esp_loader_connect(&loader, &args);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.print("Connect failed: ");
        Serial.println(loaderErrorName(err));
        targetAlreadyInBootloader = false;
        return false;
    }
    targetAlreadyInBootloader = true;

    if (!isNativeEspUsbBootloader() && espPortBaud != ESP_UART_FLASH_BAUD) {
        const esp_loader_error_t baudErr = esp_loader_change_transmission_rate(&loader, ESP_UART_FLASH_BAUD);
        if (baudErr == ESP_LOADER_SUCCESS) {
            Serial.print("UART baud: ");
            Serial.println(ESP_UART_FLASH_BAUD);
        } else {
            Serial.print("UART baud change failed: ");
            Serial.println(loaderErrorName(baudErr));
        }
    }

    return true;
}

static bool initLoaderForConnectedSerial()
{
    esp_loader_error_t err;
    if (isNativeEspUsbBootloader()) {
        err = esp_loader_init_usb(&loader, &teensyPort.base);
    } else {
        err = esp_loader_init_uart(&loader, &teensyPort.base);
    }
    if (err != ESP_LOADER_SUCCESS) {
        Serial.print("Loader init failed: ");
        Serial.println(loaderErrorName(err));
        return false;
    }
    return true;
}

static bool isNativeEspUsbBootloader()
{
    const uint16_t vid = espSerial.idVendor();
    const uint16_t pid = espSerial.idProduct();
    return vid == 0x303A && (pid == 0x1001 || pid == 0x1002);
}

static bool waitForEspUsb(uint32_t timeoutMs)
{
    const uint32_t start = millis();
    while (!espSerial && (millis() - start) < timeoutMs) {
        usbHost.Task();
        pollEspSerial();
        delay(10);
    }
    pollEspSerial();
    return (bool)espSerial;
}

static void clearEspInput()
{
    const uint32_t start = millis();
    while ((millis() - start) < 50) {
        usbHost.Task();
        while (espSerial.available()) {
            (void)espSerial.read();
        }
        delay(1);
    }
}

static bool flashFileAtOffset(const char *path, uint32_t offset)
{
    if (!sdReady) {
        Serial.println("SD not ready");
        return false;
    }
    if ((offset & 3U) != 0) {
        Serial.println("Offset must be 4-byte aligned");
        return false;
    }

    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.print("File not found: ");
        Serial.println(path);
        return false;
    }

    const uint32_t fileSize = (uint32_t)file.size();
    const uint32_t imageSize = align4(fileSize);
    Serial.print("Flashing ");
    Serial.print(path);
    Serial.print(" at 0x");
    Serial.print(offset, HEX);
    Serial.print(" size ");
    Serial.println(fileSize);

    if (!connectBootloader()) {
        file.close();
        return false;
    }

    esp_loader_flash_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.offset = offset;
    cfg.image_size = imageSize;
    cfg.block_size = FLASH_BLOCK_SIZE;
    cfg.skip_verify = false;

    esp_loader_error_t err = esp_loader_flash_start(&loader, &cfg);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.print("flash_start failed: ");
        Serial.println(loaderErrorName(err));
        file.close();
        return false;
    }

    uint32_t written = 0;
    while (written < imageSize) {
        uint32_t want = imageSize - written;
        if (want > FLASH_BLOCK_SIZE) {
            want = FLASH_BLOCK_SIZE;
        }

        uint32_t got = 0;
        if (written < fileSize) {
            uint32_t readable = fileSize - written;
            if (readable > want) {
                readable = want;
            }
            got = (uint32_t)file.read(flashBuffer, readable);
        }
        while (got < want) {
            flashBuffer[got++] = 0xFF;
        }

        err = esp_loader_flash_write(&loader, &cfg, flashBuffer, want);
        if (err != ESP_LOADER_SUCCESS) {
            Serial.print("flash_write failed at ");
            Serial.print(written);
            Serial.print(": ");
            Serial.println(loaderErrorName(err));
            file.close();
            return false;
        }

        written += want;
        const uint32_t progressPercent = imageSize > 0 ? (uint32_t)(((uint64_t)written * 100ULL) / imageSize) : 100U;
        Serial.print("Progress ");
        Serial.print(progressPercent);
        Serial.println("%");
    }

    file.close();

    err = esp_loader_flash_finish(&loader, &cfg);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.print("flash_finish failed: ");
        Serial.println(loaderErrorName(err));
        return false;
    }

    Serial.println("Flash OK");
    return true;
}

static bool flashManifest(const char *path)
{
    if (!sdReady) {
        Serial.println("SD not ready");
        return false;
    }

    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.print("Manifest not found: ");
        Serial.println(path);
        return false;
    }

    bool ok = true;
    while (file.available()) {
        uint8_t len = 0;
        while (file.available() && len < sizeof(manifestLine) - 1) {
            const char c = (char)file.read();
            if (c == '\n') {
                break;
            }
            if (c != '\r') {
                manifestLine[len++] = c;
            }
        }
        manifestLine[len] = '\0';

        char *cursor = manifestLine;
        char *offsetText = nextToken(&cursor);
        if (offsetText == NULL || offsetText[0] == '#') {
            continue;
        }
        char *fileName = nextToken(&cursor);
        uint32_t offset = 0;
        if (fileName == NULL || !parseU32(offsetText, &offset)) {
            Serial.println("Bad manifest line");
            ok = false;
            continue;
        }
        ok = flashFileAtOffset(fileName, offset) && ok;
    }

    file.close();
    Serial.println(ok ? "Manifest flash done" : "Manifest flash failed");
    return ok;
}

static bool printTargetInfo()
{
    if (!connectBootloader()) {
        return false;
    }

    const target_chip_t chip = esp_loader_get_target(&loader);
    Serial.print("Chip: ");
    Serial.println(chipName(chip));

    uint8_t mac[6] = {0};
    esp_loader_error_t err = esp_loader_read_mac(&loader, mac);
    if (err == ESP_LOADER_SUCCESS) {
        Serial.print("MAC: ");
        static const uint8_t MAC_LEN = 6;
        for (uint8_t i = 0; i < MAC_LEN; i++) {
            if (mac[i] < 16) {
                Serial.print('0');
            }
            Serial.print(mac[i], HEX);
            if ((uint8_t)(i + 1) < MAC_LEN) {
                Serial.print(':');
            }
        }
        Serial.println();
    }

    uint32_t flashSize = 0;
    err = esp_loader_flash_detect_size(&loader, &flashSize);
    if (err == ESP_LOADER_SUCCESS) {
        Serial.print("Flash size: ");
        Serial.println(flashSize);
    }

    return true;
}

static bool eraseChip()
{
    if (!connectBootloader()) {
        return false;
    }
    Serial.println("Erasing chip...");
    const esp_loader_error_t err = esp_loader_flash_erase(&loader);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.print("Erase failed: ");
        Serial.println(loaderErrorName(err));
        return false;
    }
    Serial.println("Erase OK");
    return true;
}

static void listSdRoot()
{
    if (!sdReady) {
        Serial.println("SD not ready");
        return;
    }

    File root = SD.open("/");
    if (!root) {
        Serial.println("Cannot open SD root");
        return;
    }

    File entry = root.openNextFile();
    while (entry) {
        Serial.print(entry.name());
        if (!entry.isDirectory()) {
            Serial.print(" ");
            Serial.print((uint32_t)entry.size());
        }
        Serial.println();
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
}

static void passthroughMode()
{
    Serial.println("Passthrough active. Send +++ to exit.");
    uint8_t plusCount = 0;

    while (true) {
        usbHost.Task();

        while (espSerial.available()) {
            Serial.write(espSerial.read());
        }

        while (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == '+') {
                plusCount++;
                if (plusCount >= 3) {
                    Serial.println();
                    Serial.println("Passthrough closed");
                    return;
                }
            } else {
                while (plusCount > 0) {
                    espSerial.write('+');
                    plusCount--;
                }
                espSerial.write((uint8_t)c);
            }
        }
    }
}

static char *nextToken(char **cursor)
{
    char *p = *cursor;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p == '\0') {
        *cursor = p;
        return NULL;
    }
    char *start = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        p++;
    }
    if (*p != '\0') {
        *p++ = '\0';
    }
    *cursor = p;
    return start;
}

static bool commandEquals(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        char ca = *a;
        char cb = *b;
        if (ca >= 'A' && ca <= 'Z') {
            ca = (char)(ca - 'A' + 'a');
        }
        if (cb >= 'A' && cb <= 'Z') {
            cb = (char)(cb - 'A' + 'a');
        }
        if (ca != cb) {
            return false;
        }
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static bool parseU32(const char *text, uint32_t *value)
{
    uint32_t result = 0;
    uint8_t base = 10;

    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        text += 2;
    }

    if (*text == '\0') {
        return false;
    }

    while (*text != '\0') {
        uint8_t digit;
        if (*text >= '0' && *text <= '9') {
            digit = (uint8_t)(*text - '0');
        } else if (*text >= 'a' && *text <= 'f') {
            digit = (uint8_t)(10 + *text - 'a');
        } else if (*text >= 'A' && *text <= 'F') {
            digit = (uint8_t)(10 + *text - 'A');
        } else {
            return false;
        }
        if (digit >= base) {
            return false;
        }
        result = result * base + digit;
        text++;
    }

    *value = result;
    return true;
}

static const char *chipName(target_chip_t chip)
{
    switch (chip) {
    case ESP32C3_CHIP: return "ESP32-C3";
    case ESP32S3_CHIP: return "ESP32-S3";
    case ESP32C6_CHIP: return "ESP32-C6";
    case ESP32H2_CHIP: return "ESP32-H2";
    case ESP32_CHIP: return "ESP32";
    case ESP8266_CHIP: return "ESP8266";
    default: return "unknown";
    }
}

static const char *usbSerialTypeName(USBSerialBase::sertype_t type)
{
    switch (type) {
    case USBSerialBase::CDCACM: return "CDC ACM";
    case USBSerialBase::FTDI: return "FTDI";
    case USBSerialBase::PL2303: return "PL2303";
    case USBSerialBase::CH341: return "CH340/CH341";
    case USBSerialBase::CP210X: return "CP210x";
    default: return "unknown";
    }
}

static const char *loaderErrorName(esp_loader_error_t err)
{
    switch (err) {
    case ESP_LOADER_SUCCESS: return "success";
    case ESP_LOADER_ERROR_FAIL: return "fail";
    case ESP_LOADER_ERROR_TIMEOUT: return "timeout";
    case ESP_LOADER_ERROR_IMAGE_SIZE: return "image size";
    case ESP_LOADER_ERROR_INVALID_MD5: return "invalid md5";
    case ESP_LOADER_ERROR_INVALID_PARAM: return "invalid param";
    case ESP_LOADER_ERROR_INVALID_TARGET: return "invalid target";
    case ESP_LOADER_ERROR_UNSUPPORTED_CHIP: return "unsupported chip";
    case ESP_LOADER_ERROR_UNSUPPORTED_FUNC: return "unsupported function";
    case ESP_LOADER_ERROR_INVALID_RESPONSE: return "invalid response";
    default: return "unknown";
    }
}

static uint32_t align4(uint32_t value)
{
    return (value + 3U) & ~3U;
}
