#pragma once
// =============================================================================
// pin_config.h  - LilyGo T-CAN485 hardware pin definitions
//
// IMPORTANT — CAN GPIO MUST BE VERIFIED BEFORE FLASHING.
// Open the schematic PDF at:
//   https://github.com/Xinyuan-LilyGO/T-CAN485/tree/arduino-esp32-libs_v3.0.1/project
// and confirm which GPIOs connect to the SN65HVD231 TX/RX pins.
// Known variants:
//   Some boards: TX=GPIO5  RX=GPIO4
//   Other boards: TX=GPIO27 RX=GPIO26
// =============================================================================

// ---------------------------------------------------------------------------
// CAN (TWAI) - SN65HVD231 transceiver
// TODO: Verify against YOUR board's schematic and adjust if needed.
// ---------------------------------------------------------------------------
#define PIN_CAN_TX      5     // TODO: may be GPIO27 on some board revisions
#define PIN_CAN_RX      4     // TODO: may be GPIO26 on some board revisions
#define CAN_BAUD_RATE   500000

// ---------------------------------------------------------------------------
// 5V Boost converter (ME2107A) - MUST be HIGH as first line of setup()
// Supplies the SN65HVD231 CAN transceiver. Without this, CAN won't work.
// ---------------------------------------------------------------------------
#define PIN_BOOST_EN    16

// ---------------------------------------------------------------------------
// RS-485 (MAX13487EESA - auto-direction, no DE/RE toggle required)
// ---------------------------------------------------------------------------
#define PIN_RS485_TX    22    // UART2 TX
#define PIN_RS485_RX    21    // UART2 RX
#define PIN_RS485_EN    9     // MAX13487 chip enable - drive HIGH at boot

// ---------------------------------------------------------------------------
// WS2812B RGB status LED
// NOTE: GPIO4 conflicts with PIN_CAN_RX on some board revisions.
// If your CAN_RX is GPIO4, either disable the LED or remap CAN to GPIO26/27.
// TODO: Resolve after confirming CAN GPIO variant above.
// ---------------------------------------------------------------------------
#define PIN_WS2812      4
#define WS2812_COUNT    1

// ---------------------------------------------------------------------------
// SD card (HSPI)
// ---------------------------------------------------------------------------
#define PIN_SD_MISO     2
#define PIN_SD_MOSI     15
#define PIN_SD_SCLK     14
#define PIN_SD_CS       13

// ---------------------------------------------------------------------------
// Analogue current sensing
// ADC1 only (GPIO32-36, 39) — ADC2 is blocked when WiFi is active.
// GPIO36 and GPIO39 are input-only (no internal pull-up/down).
// ---------------------------------------------------------------------------
#define PIN_CURRENT_1   36    // ADC1 CH0 (SVP)
#define PIN_CURRENT_2   39    // ADC1 CH3 (SVN)
// Current sensor calibration: ESP32 ADC is 12-bit, 0-3.3V
// Adjust CURRENT_OFFSET_MV and CURRENT_SCALE_MA_PER_MV to match your sensor
#define CURRENT_VREF_MV     3300.0f
#define CURRENT_ADC_BITS    4095.0f
#define CURRENT_OFFSET_MV   1650.0f   // sensor midpoint voltage in mV (adjust for your sensor)
#define CURRENT_SCALE_MA_PER_MV  1.0f // mA per mV (adjust for your sensor)

// ---------------------------------------------------------------------------
// Digital I/O - expansion header (choose GPIOs to suit your wiring)
// ADC1-capable: 32, 33, 34, 35. GPIO34/35 are input-only.
// GPIO25/26/27 are digital only when WiFi active (ADC2 unavailable).
// ---------------------------------------------------------------------------
#define PIN_OUT_MAIN_POS    32    // Main contactor + relay   (was Teensy OUT1)
#define PIN_OUT_MAIN_NEG    33    // Main contactor - relay   (was Teensy OUT2)
#define PIN_OUT_PRECHARGE   25    // Precharge relay           (was Teensy OUT3)
#define PIN_OUT_AUX         27    // Auxiliary/fan output      (was Teensy OUT4)
#define PIN_IN_INTERLOCK1   34    // Interlock 1 (input only)  (was Teensy IN1)
#define PIN_IN_INTERLOCK2   35    // Interlock 2 (input only)  (was Teensy IN2)

// ---------------------------------------------------------------------------
// BOOT button - doubles as user input
// ---------------------------------------------------------------------------
#define PIN_BOOT_BTN    0
