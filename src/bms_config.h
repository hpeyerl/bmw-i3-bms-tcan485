#pragma once
// =============================================================================
// bms_config.h  - BMW i3 CSC BMS on LilyGo T-CAN485
//
// Target: LilyGo T-CAN485 (ESP32, 4 MB flash, SN65HVD231 CAN, MAX13487 RS485)
// CMU:    BMW i3 CSC modules via CAN bus only.
//         Tesla UART path has been removed.
//
// Credits:
//   Collin Kidder  (collin80/TeslaBMS)         - original BMS architecture, MIT
//   Tom-evnut      (Tom-evnut/BMWPhevBMS)       - BMW i3 CSC CAN protocol, MIT
//   ronaegis       (ronaegis/tesla-bms-esp32s3) - ESP32-S3 TWAI port pattern, MIT
// =============================================================================

// ---------------------------------------------------------------------------
// Pack topology defaults (overridden at runtime via serial console or WiFi)
// ---------------------------------------------------------------------------
#define BMS_NUM_SERIES           8      // BMW i3 modules in series (typical full pack)
#define BMS_NUM_PARALLEL         1      // Strings in parallel

// ---------------------------------------------------------------------------
// Module address range: BMW i3 CSC supports up to 8 modules (addresses 1-8)
// ---------------------------------------------------------------------------
#define MAX_MODULE_ADDR          8      // BMW i3 max CSC count

// ---------------------------------------------------------------------------
// BMW i3 CSC CAN IDs
//
// Each CSC broadcasts three sub-frames per scan cycle:
//   0x3D1+n  cell voltages  (4 cells per sub-frame, 2 bytes each, 1 mV/bit, big-endian)
//   0x3B1+n  temperatures   (2 bytes each, 0.1 degC/bit, signed 16-bit, big-endian)
//
// Module index n = 0..7  (CSC address 1..8 maps to IDs 0x3D1..0x3D8 / 0x3B1..0x3B8)
//
// Enumeration / management frames:
//   0x0A0  broadcast commands (find unassigned, assign ID, reset)
//   0x0B0  balance reset broadcast
//   0x130  wake frame (sent by master at startup)
// ---------------------------------------------------------------------------
#define BMW_I3_CELL_BASE        0x3D1
#define BMW_I3_TEMP_BASE        0x3B1
#define BMW_I3_WAKE_ID          0x130
#define BMW_I3_CMD_ID           0x0A0
#define BMW_I3_BAL_RESET_ID     0x0B0
#define BMW_I3_CELLS_PER_MOD    12      // 12 cells per CSC module
#define BMW_I3_MAX_MODS         8       // max 8 CSC in one pack

// ---------------------------------------------------------------------------
// CAN-based charger/inverter heartbeat detection
// Balancing is paused while a charger or inverter heartbeat is not seen.
// Default ID: 0x305 (LEM CAB300 current sensor / Victron heartbeat)
// ---------------------------------------------------------------------------
#define CHARGER_TIMEOUT_MS          5000
#define DEFAULT_CHARGER_HB_ID       0x305
#define DEFAULT_CAN_INHIBIT         0     // 0 = off at factory defaults

// ---------------------------------------------------------------------------
// WiFi defaults
// ---------------------------------------------------------------------------
#define WIFI_SSID_DEFAULT       "BMWI3BMS"
#define WIFI_PASS_DEFAULT       "bmwpack1"
#define WIFI_AP_MODE            true

// ---------------------------------------------------------------------------
// NVS / settings version.  Bump whenever EEPROMSettings layout changes.
// A mismatch triggers a factory-default reset on boot.
// ---------------------------------------------------------------------------
#define EEPROM_VERSION          0x20    // T-CAN485 port initial version
#define EEPROM_PAGE             0

// ---------------------------------------------------------------------------
// Cell voltage / temperature defaults
// ---------------------------------------------------------------------------
#define DEFAULT_OVER_V          4.20f
#define DEFAULT_UNDER_V         3.00f
#define DEFAULT_OVER_T          55.0f
#define DEFAULT_UNDER_T        -20.0f
#define DEFAULT_CHARGE_T        45.0f
#define DEFAULT_DIS_T          -20.0f
#define DEFAULT_BALANCE_V       4.10f
#define DEFAULT_BALANCE_HYST    0.02f
#define DEFAULT_IGNORE_VOLT     0.0f    // BMW i3 cells: no unpopulated cells, set 0
#define DEFAULT_IGNORE_TEMP     0
#define DEFAULT_IGNORE_TEMP_THRESH  -70.0f

// BMW i3 SoC calibration (per-module voltage range)
// Typical i3 cell: 2.5V (empty) to 4.15V (full), 12 cells per module
// Per-module range: 30.0V - 49.8V
#define DEFAULT_NUM_CELLS       12
#define DEFAULT_SOC_LO          30.0f   // V per module at 0%  SoC
#define DEFAULT_SOC_HI          49.8f   // V per module at 100% SoC

// ---------------------------------------------------------------------------
// Serial port
// ---------------------------------------------------------------------------
#include <Arduino.h>
#define SERIALCONSOLE   Serial   // USB via CH9102

// ---------------------------------------------------------------------------
// settingsSave()
// Always call this instead of raw EEPROM.put()+commit().
// Updates the XOR checksum before writing so loadSettings() can detect
// corruption or partial writes on the next boot.
// ---------------------------------------------------------------------------
#include <EEPROM.h>

// ---------------------------------------------------------------------------
// EEPROMSettings struct
// Stored in ESP32 NVS via arduino-esp32 EEPROM emulation.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  version;
    uint8_t  checksum;
    uint32_t canSpeed;
    uint8_t  batteryID;
    uint8_t  logLevel;
    float    OverVSetpoint;
    float    UnderVSetpoint;
    float    OverTSetpoint;
    float    UnderTSetpoint;
    float    ChargeTSetpoint;
    float    DisTSetpoint;
    uint8_t  IgnoreTemp;
    float    IgnoreVolt;
    float    balanceVoltage;
    float    balanceHyst;
    float    IgnoreTempThresh;
    uint8_t  wifiEnabled;
    uint8_t  balancingEnabled;
    char     wifiSSID[32];
    char     wifiPass[32];
    uint8_t  numCells;       // cells per module (always 12 for BMW i3)
    uint8_t  numSeries;      // modules in series
    uint8_t  numParallel;    // strings in parallel
    float    socLo;          // V per module at 0% SoC
    float    socHi;          // V per module at 100% SoC
    // CAN-based charger inhibit
    uint8_t  canInhibitEnabled;
    uint32_t chargerHeartbeatID;
    uint8_t  batteryID_u8;   // redundant field kept for struct padding alignment
} EEPROMSettings;

inline uint8_t settingsComputeChecksum(const EEPROMSettings &s)
{
    const uint8_t *p    = reinterpret_cast<const uint8_t *>(&s);
    const uint8_t *end  = p + sizeof(EEPROMSettings);
    const uint8_t *skip = reinterpret_cast<const uint8_t *>(&s.checksum);
    uint8_t xorSum = 0;
    for (; p < end; ++p) { if (p != skip) xorSum ^= *p; }
    return xorSum;
}
inline void settingsSave(EEPROMSettings &s)
{
    s.checksum = settingsComputeChecksum(s);
    EEPROM.put(EEPROM_PAGE, s);
    EEPROM.commit();
}
