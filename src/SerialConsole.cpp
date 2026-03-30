// =============================================================================
// SerialConsole.cpp - USB serial debug interface for BMW i3 / T-CAN485
//
// Changes from M5Dial version:
//   - Removed Nextion / display commands
//   - Removed Tesla UART commands (S/W/C/F/R)
//   - Added BMW i3 enumeration commands: E (find), A (assign), Z (reset IDs)
//   - Removed CMUTYPE setting (always BMW i3 on this build)
//   - Settings saved via EEPROM (arduino-esp32 NVS emulation, same API)
//   - SOC save command 'V' to manually persist SOC to NVS
// =============================================================================
#include "SerialConsole.h"
#include "Logger.h"
#include "BMSModuleManager.h"
#include "CANManager.h"
#include <EEPROM.h>

extern EEPROMSettings   settings;
extern BMSModuleManager bms;
extern CANManager       can;

// Runtime display toggle state
static bool     printPrettyDisplay = false;
static uint32_t prettyCounter      = 0;
static int      whichDisplay       = 0;  // 0=summary, 1=details

SerialConsole::SerialConsole() { init(); }

void SerialConsole::init()
{
    ptrBuffer          = 0;
    state              = STATE_ROOT_MENU;
    loopcount          = 0;
    cancel             = false;
    printPrettyDisplay = false;
    prettyCounter      = 0;
    whichDisplay       = 0;
}

void SerialConsole::loop()
{
    if (SERIALCONSOLE.available()) serialEvent();
    if (printPrettyDisplay && (millis() > prettyCounter + 3000)) {
        prettyCounter = millis();
        if (whichDisplay == 0) bms.printPackSummary();
        else                   bms.printPackDetails();
    }
}

// ---------------------------------------------------------------------------
// printMenu
// ---------------------------------------------------------------------------
void SerialConsole::printMenu()
{
    Logger::console("\n========== BMW i3 BMS T-CAN485 - MENU ==========");
    Logger::console("Line ending required (LF, CR, or CRLF)\n");

    Logger::console("--- Pack Info ---");
    Logger::console("  h / ?      This menu");
    Logger::console("  p          Toggle pack summary (every 3s)");
    Logger::console("  d          Toggle pack details (every 3s)");
    Logger::console("  B          Manual balance inhibit toggle\n");

    Logger::console("--- BMW i3 CSC Enumeration ---");
    Logger::console("  E          Find unassigned CSC (send 0x0A0 query)");
    Logger::console("  A          Assign next ID to found unassigned CSC");
    Logger::console("  Z          Reset all CSC IDs (use before re-enumeration)");
    Logger::console("  K          Send balance reset frame (0x0B0)");
    Logger::console("  W          Send wake frame (0x130)\n");

    Logger::console("--- CAN ---");
    Logger::console("  X          Stop CAN TX+RX");
    Logger::console("  x          Restart CAN TX+RX\n");

    Logger::console("--- Pack Configuration ---");
    Logger::console("  NUMSERIES=N      Modules in series [%d]", settings.numSeries);
    Logger::console("  NUMPARALLEL=N    Strings in parallel [%d]", settings.numParallel);
    Logger::console("  SOCLO=N.NN       V/module at 0%% SoC [%.2f]", settings.socLo);
    Logger::console("  SOCHI=N.NN       V/module at 100%% SoC [%.2f]\n", settings.socHi);

    Logger::console("--- Cell Thresholds ---");
    Logger::console("  VOLTLIMHI=N.NN   OV limit per cell [%.2fV]", settings.OverVSetpoint);
    Logger::console("  VOLTLIMLO=N.NN   UV limit per cell [%.2fV]", settings.UnderVSetpoint);
    Logger::console("  TEMPLIMHI=N.NN   Over-temp limit [%.1fC]", settings.OverTSetpoint);
    Logger::console("  TEMPLIMLO=N.NN   Under-temp limit [%.1fC]", settings.UnderTSetpoint);
    Logger::console("  IGNORETEMP=N.NN  Ignore temps below this C [%.1fC]\n", settings.IgnoreTempThresh);

    Logger::console("--- Connectivity ---");
    Logger::console("  WIFI=0/1         WiFi at boot [%s]", settings.wifiEnabled ? "ON" : "OFF");
    Logger::console("  BATTERYID=N      SimpBMS CAN battery ID [%d]", settings.batteryID);
    Logger::console("  LOGLEVEL=N       0=debug 1=info 2=warn 3=error 4=off [%d]", settings.logLevel);
    Logger::console("  CANINHIBIT=0/1   CAN-based balance inhibit [%s]", settings.canInhibitEnabled ? "ON" : "OFF");
    Logger::console("  CHGID=0xNNN      Charger heartbeat CAN ID [0x%03X]", settings.chargerHeartbeatID);
    Logger::console("  CSCVARIANT=0/1/2 CSC type: 0=BMW i3, 1=Mini-E, 2=BMWI3BUS [%d]\n", settings.CSCvariant);

    Logger::console("=================================================\n");
}

// ---------------------------------------------------------------------------
// serialEvent / handleConsoleCmd
// ---------------------------------------------------------------------------
void SerialConsole::serialEvent()
{
    int incoming = SERIALCONSOLE.read();
    if (incoming == -1) return;
    if (incoming == 10 || incoming == 13) {
        handleConsoleCmd();
        ptrBuffer = 0;
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char)incoming;
        if (ptrBuffer > 80) ptrBuffer = 80;
    }
}

void SerialConsole::handleConsoleCmd()
{
    if (ptrBuffer == 1) handleShortCmd();
    else                handleConfigCmd();
}

// ---------------------------------------------------------------------------
// handleShortCmd
// ---------------------------------------------------------------------------
void SerialConsole::handleShortCmd()
{
    switch (cmdBuffer[0]) {

    case 'h': case '?': case 'H':
        printMenu();
        break;

    case 'p':
        if (whichDisplay == 1 && printPrettyDisplay) { whichDisplay = 0; }
        else {
            printPrettyDisplay = !printPrettyDisplay;
            whichDisplay = 0;
            Logger::console(printPrettyDisplay ? "Pack summary ON (3s)" : "Pack summary OFF");
        }
        break;

    case 'd':
        if (whichDisplay == 0 && printPrettyDisplay) { whichDisplay = 1; }
        else {
            printPrettyDisplay = !printPrettyDisplay;
            whichDisplay = 1;
            Logger::console(printPrettyDisplay ? "Pack details ON (3s)" : "Pack details OFF");
        }
        break;

    case 'B':
        {
            bool inh = !bms.getBalanceInhibit();
            bms.setBalanceInhibit(inh);
            Logger::console("Balance inhibit: %s", inh ? "ON (manual)" : "OFF (manual)");
        }
        break;

    // --- BMW i3 enumeration commands ---
    case 'E':
        Logger::console("Sending find-unassigned query (0x0A0 / 0x37)...");
        can.sendI3FindUnassigned();
        Logger::console("Watch for 0x4A0 reply. Then press A to assign ID.");
        break;

    case 'A':
        if (can.hasUnassignedCSC()) {
            // Find next free module slot (1..8)
            int nextID = -1;
            for (int i = 1; i <= BMW_I3_MAX_MODS; i++) {
                if (!bms.getModule(i).isExisting()) { nextID = i; break; }
            }
            if (nextID < 0) {
                Logger::console("No free module slots (all %d in use)", BMW_I3_MAX_MODS);
            } else {
                Logger::console("Assigning ID %d to unassigned CSC...", nextID);
                can.sendI3AssignID((uint8_t)nextID, can.getUnassignedDMC());
                can.clearUnassignedFlag();
                Logger::console("Done. Press E to find the next unassigned CSC.");
            }
        } else {
            Logger::console("No unassigned CSC found. Press E first.");
        }
        break;

    case 'Z':
        Logger::console("Resetting all CSC IDs (broadcast 0..14)...");
        can.sendI3ResetAllIDs(14);
        Logger::console("Done. Press E then A to enumerate from scratch.");
        break;

    case 'K':
        Logger::console("Sending balance reset (0x0B0)...");
        can.sendI3BalanceReset();
        break;

    case 'W':
        Logger::console("Sending wake frame (0x130)...");
        can.sendI3WakeFrame();
        break;

    case 'X':
        Logger::console("Stopping CAN...");
        can.end();
        break;

    case 'x':
        Logger::console("Starting CAN...");
        can.begin();
        break;
    }
}

// ---------------------------------------------------------------------------
// handleConfigCmd - KEY=value commands
// ---------------------------------------------------------------------------
void SerialConsole::handleConfigCmd()
{
    int   newValue;
    float newFloat;
    bool  needSave = false;

    if (ptrBuffer < 3) return;
    cmdBuffer[ptrBuffer] = 0;

    String cmdString;
    int i = 0;
    while (cmdBuffer[i] != '=' && i < ptrBuffer) cmdString.concat((char)(cmdBuffer[i++]));
    i++;  // skip =
    if (i >= ptrBuffer) { Logger::console("Format: KEY=value"); return; }

    newValue = strtol((char *)(cmdBuffer + i), NULL, 0);
    newFloat = strtof((char *)(cmdBuffer + i), NULL);
    cmdString.toUpperCase();

    if (cmdString == "NUMSERIES") {
        if (newValue >= 1 && newValue <= 20) {
            settings.numSeries = (uint8_t)newValue;
            bms.setPstrings(settings.numParallel);
            needSave = true;
            Logger::console("Modules in series: %d", newValue);
        } else Logger::console("Invalid (1-20)");
    }
    else if (cmdString == "NUMPARALLEL") {
        if (newValue >= 1 && newValue <= 4) {
            settings.numParallel = (uint8_t)newValue;
            bms.setPstrings(newValue);
            needSave = true;
            Logger::console("Strings in parallel: %d", newValue);
        } else Logger::console("Invalid (1-4)");
    }
    else if (cmdString == "SOCLO") {
        if (newFloat >= 10.0f && newFloat < settings.socHi) {
            settings.socLo = newFloat;
            needSave = true;
            Logger::console("SoC 0%% V/module: %.2f", newFloat);
        } else Logger::console("Invalid");
    }
    else if (cmdString == "SOCHI") {
        if (newFloat > settings.socLo && newFloat <= 60.0f) {
            settings.socHi = newFloat;
            needSave = true;
            Logger::console("SoC 100%% V/module: %.2f", newFloat);
        } else Logger::console("Invalid");
    }
    else if (cmdString == "VOLTLIMHI") {
        if (newFloat >= 3.0f && newFloat <= 5.0f) {
            settings.OverVSetpoint = newFloat;
            needSave = true;
            Logger::console("OV limit: %.2fV", newFloat);
        } else Logger::console("Invalid (3.0-5.0)");
    }
    else if (cmdString == "VOLTLIMLO") {
        if (newFloat >= 2.0f && newFloat <= 4.0f) {
            settings.UnderVSetpoint = newFloat;
            needSave = true;
            Logger::console("UV limit: %.2fV", newFloat);
        } else Logger::console("Invalid (2.0-4.0)");
    }
    else if (cmdString == "TEMPLIMHI") {
        if (newFloat >= 20.0f && newFloat <= 80.0f) {
            settings.OverTSetpoint = newFloat;
            needSave = true;
            Logger::console("OT limit: %.1fC", newFloat);
        } else Logger::console("Invalid (20-80)");
    }
    else if (cmdString == "TEMPLIMLO") {
        if (newFloat >= -40.0f && newFloat <= 20.0f) {
            settings.UnderTSetpoint = newFloat;
            needSave = true;
            Logger::console("UT limit: %.1fC", newFloat);
        } else Logger::console("Invalid (-40 to 20)");
    }
    else if (cmdString == "IGNORETEMP") {
        if (newFloat >= -100.0f && newFloat <= 0.0f) {
            settings.IgnoreTempThresh = newFloat;
            needSave = true;
            Logger::console("Ignore temps below: %.1fC", newFloat);
        } else Logger::console("Invalid (-100 to 0)");
    }
    else if (cmdString == "WIFI") {
        settings.wifiEnabled = (newValue != 0) ? 1 : 0;
        needSave = true;
        Logger::console("WiFi at boot: %s (reboot to apply)", settings.wifiEnabled ? "ON" : "OFF");
    }
    else if (cmdString == "BATTERYID") {
        if (newValue >= 1 && newValue <= 14) {
            settings.batteryID = (uint8_t)newValue;
            bms.setBatteryID(newValue);
            needSave = true;
            Logger::console("Battery CAN ID: %d", newValue);
        } else Logger::console("Invalid (1-14)");
    }
    else if (cmdString == "LOGLEVEL") {
        if (newValue >= 0 && newValue <= 4) {
            Logger::setLoglevel((Logger::LogLevel)newValue);
            settings.logLevel = (uint8_t)newValue;
            needSave = true;
            const char *names[] = {"DEBUG","INFO","WARN","ERROR","OFF"};
            Logger::console("Log level: %s", names[newValue]);
        } else Logger::console("Invalid (0-4)");
    }
    else if (cmdString == "CANINHIBIT") {
        settings.canInhibitEnabled = (newValue != 0) ? 1 : 0;
        needSave = true;
        Logger::console("CAN inhibit: %s", settings.canInhibitEnabled ? "ON" : "OFF");
    }
    else if (cmdString == "CHGID") {
        uint32_t id = (uint32_t)strtoul((const char *)(cmdBuffer + i), NULL, 0);
        if (id > 0 && id <= 0x7FF) {
            settings.chargerHeartbeatID = id;
            needSave = true;
            Logger::console("Charger heartbeat ID: 0x%03X", id);
        } else Logger::console("Invalid CAN ID (0x001-0x7FF)");
    }
    else if (cmdString == "CSCVARIANT") {
        if (newValue >= CSC_VARIANT_BMWI3 && newValue <= CSC_VARIANT_BMWI3BUS) {
            settings.CSCvariant = (uint8_t)newValue;
            needSave = true;
            const char *names[] = {"0=BMW i3 (0x3D1-0x3D8)", "1=Mini-E (0x0A0-0x15F)", "2=BMWI3BUS (0x100-0x15F)"};
            Logger::console("CSC variant set: %s", names[newValue]);
        } else Logger::console("Invalid: 0=BMW i3, 1=Mini-E, 2=BMWI3BUS");
    }
    else {
        Logger::console("Unknown: '%s'  Press H for menu.", cmdString.c_str());
    }

    if (needSave) {
        settingsSave(settings);
        Logger::console("Saved.");
    }
}
