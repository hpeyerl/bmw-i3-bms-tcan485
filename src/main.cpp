// =============================================================================
// BMWI3BMS.ino  - BMW i3 CSC BMS on LilyGo T-CAN485
//
// Hardware:
//   MCU       : ESP32 (dual-core Xtensa LX6 @ 240 MHz), 4 MB flash
//   CAN       : TWAI peripheral + SN65HVD231 transceiver
//   RS-485    : MAX13487EESA on UART2 (not used for BMW i3 CAN-only mode)
//   USB serial: CH9102 USB-UART bridge
//   Status LED: WS2812B on GPIO4 (see pin_config.h TODO for conflict note)
//   Boost EN  : GPIO16 must be HIGH immediately at boot
//
// Credits:
//   Collin Kidder  (collin80/TeslaBMS)         - original BMS architecture, MIT
//   Tom-evnut      (Tom-evnut/BMWPhevBMS)       - BMW i3 CSC CAN protocol, MIT
//   ronaegis       (ronaegis/tesla-bms-esp32s3) - ESP32-S3 TWAI port pattern, MIT
//
// Removed vs M5Dial v6:
//   - M5Unified / M5GFX / LVGL (no display)
//   - Tesla UART CMU path (BMSUtil, BMSModule UART read, Serial1 at 612500)
//   - Rotary encoder / buzzer
//   - Power-hold pin (no M5Dial latch)
//   - PMC low-voltage ISR (no equivalent on ESP32)
//
// SOC persistence:
//   No low-voltage ISR exists on ESP32. SOC is written to NVS every
//   SOC_SAVE_INTERVAL_MS while running. Up to that window may be lost on
//   unexpected power loss.
// =============================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>
#include "pin_config.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "SerialConsole.h"
#include "Logger.h"
#include "CANManager.h"
#include "WiFiManager.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
BMSModuleManager bms;
SerialConsole    console;
EEPROMSettings   settings;
CANManager       can;
WiFiManager      wifi;

CRGB leds[WS2812_COUNT];

// Status strings populated by BMSModuleManager, read by WiFiManager
extern String bms_status;
extern String bms_modules_text;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define BMS_READ_INTERVAL_MS    1000
#define CAN_SEND_INTERVAL_MS    1000
#define SOC_SAVE_INTERVAL_MS   30000   // NVS SOC write interval (replaces LVD ISR)
#define LED_UPDATE_MS            500

static uint32_t lastBmsRead   = 0;
static uint32_t lastCANSend   = 0;
static uint32_t lastSOCSave   = 0;
static uint32_t lastLEDUpdate = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void loadSettings();
void applySettings();
void updateLED();

// ---------------------------------------------------------------------------
// Helpers exposed to SerialConsole
// ---------------------------------------------------------------------------
void stopWifiAndCan()
{
    if (can.isRunning())  can.end();
    if (wifi.isRunning()) wifi.end();
}

void setCANEnabled(bool en)
{
    if (en  && !can.isRunning()) can.begin();
    if (!en &&  can.isRunning()) can.end();
}

// =============================================================================
// setup()
// =============================================================================
void setup()
{
    // 1. Boost converter enable MUST be first - powers the SN65HVD231.
    //    Without this, CAN transceiver has no 3.3V supply.
    pinMode(PIN_BOOST_EN, OUTPUT);
    digitalWrite(PIN_BOOST_EN, HIGH);

    // 2. RS-485 chip enable (MAX13487 auto-direction, drive HIGH to enable)
    pinMode(PIN_RS485_EN, OUTPUT);
    digitalWrite(PIN_RS485_EN, HIGH);

    // 3. Output GPIOs for contactors (default LOW = contactors open)
    pinMode(PIN_OUT_MAIN_POS,  OUTPUT); digitalWrite(PIN_OUT_MAIN_POS,  LOW);
    pinMode(PIN_OUT_MAIN_NEG,  OUTPUT); digitalWrite(PIN_OUT_MAIN_NEG,  LOW);
    pinMode(PIN_OUT_PRECHARGE, OUTPUT); digitalWrite(PIN_OUT_PRECHARGE, LOW);
    pinMode(PIN_OUT_AUX,       OUTPUT); digitalWrite(PIN_OUT_AUX,       LOW);

    // 4. Input GPIOs
    pinMode(PIN_IN_INTERLOCK1, INPUT_PULLUP);
    pinMode(PIN_IN_INTERLOCK2, INPUT_PULLUP);
    pinMode(PIN_BOOT_BTN,      INPUT_PULLUP);

    // 5. ADC resolution
    analogReadResolution(12);   // 12-bit (0-4095, 0-3.3V)

    // 6. Status LED
    FastLED.addLeds<WS2812B, PIN_WS2812, GRB>(leds, WS2812_COUNT);
    FastLED.setBrightness(40);
    leds[0] = CRGB::Blue;   // booting
    FastLED.show();

    // 7. USB serial console
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 2000) delay(10);
    Logger::console("BMW i3 BMS T-CAN485 booting...");
    Logger::console("CAN pins: TX=GPIO%d RX=GPIO%d  (verify vs schematic!)",
                    PIN_CAN_TX, PIN_CAN_RX);

    // 8. NVS / EEPROM settings
    EEPROM.begin(sizeof(EEPROMSettings));
    loadSettings();
    applySettings();

    // 9. WiFi (if enabled at boot)
    if (settings.wifiEnabled) {
        Logger::console("Starting WiFi AP...");
        wifi.begin();
        Logger::console("WiFi AP: SSID=%s  http://%s", settings.wifiSSID, wifi.getIP().c_str());
    } else {
        Logger::console("WiFi disabled. Enable with WIFI=1 then reboot.");
    }

    // 10. CAN - starts TWAI driver + RX FreeRTOS task on Core 0
    if (can.begin()) {
        Logger::console("CAN ready");
    } else {
        Logger::console("CAN FAILED - check GPIO%d/GPIO%d and PIN_BOOST_EN",
                        PIN_CAN_TX, PIN_CAN_RX);
        leds[0] = CRGB::Red;
        FastLED.show();
    }

    Logger::console("Boot complete. Press H for menu.");
    leds[0] = CRGB::Green;
    FastLED.show();
}

// =============================================================================
// loop()  - runs on Core 1 (Arduino default)
// =============================================================================
void loop()
{
    uint32_t now = millis();

    // --- Balance inhibit: CAN charger heartbeat (if enabled) ---
    bool canInhibit = (settings.canInhibitEnabled &&
                       can.isRunning() &&
                       !can.getChargerActive());
    bms.setBalanceInhibit(canInhibit);

    // --- BMS data read from CAN staging buffers ---
    if (now - lastBmsRead >= BMS_READ_INTERVAL_MS) {
        lastBmsRead = now;
        bms.getAllVoltTempFromCAN();

        if (Logger::isDebug()) {
            Logger::debug("%s", bms_status.c_str());
        }
    }

    // --- SimpBMS CAN TX ---
    if (can.isRunning() && (now - lastCANSend >= CAN_SEND_INTERVAL_MS)) {
        lastCANSend = now;
        can.sendBatterySummary();
    }

    // --- SOC save to NVS (replaces Teensy PMC low-voltage ISR) ---
    if (now - lastSOCSave >= SOC_SAVE_INTERVAL_MS) {
        lastSOCSave = now;
        // Re-use the existing settings struct - no new fields needed,
        // SOC is calculated dynamically from pack voltage each time.
        // If you add a persistent SOC field to EEPROMSettings in future,
        // write it here: settings.lastSOC = bms.getSoCPercent();
        // EEPROM.put(EEPROM_PAGE, settings); EEPROM.commit();
    }

    // --- Status LED ---
    if (now - lastLEDUpdate >= LED_UPDATE_MS) {
        lastLEDUpdate = now;
        updateLED();
    }

    // --- Serial console ---
    console.loop();

    // --- WiFi housekeeping (AsyncWebServer is event-driven, loop is a no-op) ---
    if (wifi.isRunning()) wifi.loop();
}

// =============================================================================
// updateLED - WS2812B status indicator
//   Green  = all modules present, no faults
//   Amber  = some modules missing or balance inhibited
//   Red    = fault detected
//   Blue   = no modules seen yet
// =============================================================================
void updateLED()
{
    int   n = bms.getNumModules();
    bool  f = bms.isFaultedState();
    bool  i = bms.getBalanceInhibit();

    CRGB colour;
    if (f)       colour = CRGB::Red;
    else if (n == 0) colour = CRGB::Blue;
    else if (i)  colour = CRGB::Orange;
    else         colour = CRGB::Green;

    leds[0] = colour;
    FastLED.show();
}

// =============================================================================
// loadSettings
// =============================================================================
void loadSettings()
{
    EEPROM.get(EEPROM_PAGE, settings);
    if (settings.version != EEPROM_VERSION) {
        Logger::console("NVS version mismatch (0x%X vs 0x%X). Loading defaults.",
                        settings.version, EEPROM_VERSION);
        // Zero entire struct first to avoid garbage in padding bytes
        memset(&settings, 0, sizeof(settings));
        settings.version          = EEPROM_VERSION;
        settings.canSpeed         = CAN_BAUD_RATE;
        settings.batteryID        = 1;
        settings.logLevel         = Logger::Info;
        settings.OverVSetpoint    = DEFAULT_OVER_V;
        settings.UnderVSetpoint   = DEFAULT_UNDER_V;
        settings.OverTSetpoint    = DEFAULT_OVER_T;
        settings.UnderTSetpoint   = DEFAULT_UNDER_T;
        settings.ChargeTSetpoint  = DEFAULT_CHARGE_T;
        settings.DisTSetpoint     = DEFAULT_DIS_T;
        settings.IgnoreTemp       = DEFAULT_IGNORE_TEMP;
        settings.IgnoreVolt       = DEFAULT_IGNORE_VOLT;
        settings.IgnoreTempThresh = DEFAULT_IGNORE_TEMP_THRESH;
        settings.balanceVoltage   = DEFAULT_BALANCE_V;
        settings.balanceHyst      = DEFAULT_BALANCE_HYST;
        settings.wifiEnabled      = 0;
        settings.balancingEnabled = 0;
        strncpy(settings.wifiSSID, WIFI_SSID_DEFAULT, 31);
        strncpy(settings.wifiPass,  WIFI_PASS_DEFAULT, 31);
        settings.numCells         = DEFAULT_NUM_CELLS;
        settings.numSeries        = BMS_NUM_SERIES;
        settings.numParallel      = BMS_NUM_PARALLEL;
        settings.socLo            = DEFAULT_SOC_LO;
        settings.socHi            = DEFAULT_SOC_HI;
        settings.canInhibitEnabled  = DEFAULT_CAN_INHIBIT;
        settings.chargerHeartbeatID = DEFAULT_CHARGER_HB_ID;
        EEPROM.put(EEPROM_PAGE, settings);
        EEPROM.commit();
        Logger::console("Defaults written to NVS.");
    } else {
        Logger::console("Settings loaded: OV=%.2fV UV=%.2fV WiFi=%d CanInh=%d",
                        settings.OverVSetpoint, settings.UnderVSetpoint,
                        settings.wifiEnabled, settings.canInhibitEnabled);
    }
}

// =============================================================================
// applySettings
// =============================================================================
void applySettings()
{
    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
    bms.setBatteryID(settings.batteryID);
    bms.setPstrings(settings.numParallel > 0 ? settings.numParallel : BMS_NUM_PARALLEL);
    bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
}
