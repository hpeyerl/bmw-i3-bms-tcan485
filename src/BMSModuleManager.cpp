// Credits:
//   Tom-evnut      (Tom-evnut/BMWPhevBMS)       - BMW i3 CSC CAN protocol, MIT
// =============================================================================
// BMSModuleManager.cpp - Pack-level BMS manager (BMW i3 CAN only)
//
// getAllVoltTempFromCAN() reads from CANManager's I3SlaveData staging buffers
// and populates BMSModule objects.  Fault detection is threshold-based
// (OV/UV/OT/UT) since BMW i3 CSC modules do not report a fault register.
// =============================================================================
#include "BMSModuleManager.h"
#include "Logger.h"
#include "bms_config.h"
#include <EEPROM.h>

extern EEPROMSettings settings;
extern CANManager     can;

// Global status strings populated here; read by WiFiManager for JSON output
String bms_status;
String bms_modules_text;

BMSModuleManager::BMSModuleManager()
{
    for (int i = 0; i <= MAX_MODULE_ADDR; i++) {
        modules[i].setExists(false);
        modules[i].setAddress(i);
        modules[i].setNumCells(BMW_I3_CELLS_PER_MOD);
    }
    packVolt        = 0.0f;
    lowCellVolt     = 5.0f;
    highCellVolt    = 0.0f;
    lowestPackVolt  = 1000.0f;
    highestPackVolt = 0.0f;
    numFoundModules = 0;
    Pstring         = BMS_NUM_PARALLEL;
    batteryID       = 1;
    isFaulted       = false;
    balanceInhibit  = false;
}

// ---------------------------------------------------------------------------
// getAllVoltTempFromCAN
// Reads I3SlaveData staging buffers from CANManager and populates BMSModule.
// Modules are marked alive only if last seen within 3 s.
// ---------------------------------------------------------------------------
void BMSModuleManager::getAllVoltTempFromCAN()
{
    packVolt        = 0.0f;
    lowCellVolt     = 9999.0f;
    highCellVolt    = 0.0f;
    numFoundModules = 0;

    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        I3SlaveData d;
        if (!can.getI3SlaveData(x, d)) {
            // No data ever received; mark not existing
            modules[x].setExists(false);
            continue;
        }

        bool alive = (millis() - d.lastSeenMs) < 3000;
        modules[x].setExists(alive);
        if (!alive) continue;

        numFoundModules++;
        modules[x].setNumCells(BMW_I3_CELLS_PER_MOD);

        float modV = 0.0f;
        for (int c = 0; c < BMW_I3_CELLS_PER_MOD; c++) {
            modules[x].setCellVoltage(c, d.cellV[c]);
            modV += d.cellV[c];
        }
        modules[x].setModuleVoltage(modV);
        modules[x].setTemperature(0, d.temp[0]);
        modules[x].setTemperature(1, d.temp[1]);
        // Faults detected by checkFaults() against thresholds
        modules[x].setFaults(0);
        modules[x].setAlerts(0);

        packVolt += modV;

        float lo = modules[x].getLowCellV();
        float hi = modules[x].getHighCellV();
        if (lo > 0.0f && lo < lowCellVolt)  lowCellVolt  = lo;
        if (hi > highCellVolt)               highCellVolt = hi;
    }

    // Apply parallel string divisor
    if (Pstring > 1) packVolt /= Pstring;

    if (packVolt > highestPackVolt) highestPackVolt = packVolt;
    if (packVolt < lowestPackVolt && packVolt > 0.0f) lowestPackVolt = packVolt;

    // After populating, run threshold fault detection
    checkFaults();

    // Update status strings for WiFi/serial output
    char buf[128];
    snprintf(buf, sizeof(buf),
             "SoC:%.0f%% Pack:%.2fV Lo:%.3fV Hi:%.3fV dV:%.0fmV Mods:%d %s",
             getSoCPercent(), packVolt, lowCellVolt, highCellVolt,
             (highCellVolt - lowCellVolt) * 1000.0f,
             numFoundModules,
             isFaulted ? "FAULTED" : "OK");
    bms_status = String(buf);

    bms_modules_text = "";
    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        if (modules[x].isExisting()) {
            snprintf(buf, sizeof(buf),
                     "Mod%d: %.2fV lo:%.3fV hi:%.3fV T:%.1f/%.1fC\n",
                     x, modules[x].getModuleVoltage(),
                     modules[x].getLowCellV(), modules[x].getHighCellV(),
                     modules[x].getTemperature(0), modules[x].getTemperature(1));
            bms_modules_text += buf;
        }
    }
}

// ---------------------------------------------------------------------------
// checkFaults - threshold-based fault detection for BMW i3
// Sets module fault/alert flags so the WiFi dashboard and SimpBMS CAN TX
// can report them, even though the CSC doesn't have a fault register.
// ---------------------------------------------------------------------------
void BMSModuleManager::checkFaults()
{
    isFaulted = false;
    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        if (!modules[x].isExisting()) continue;
        uint8_t f = 0, a = 0;
        // OV/UV per cell
        for (int c = 0; c < BMW_I3_CELLS_PER_MOD; c++) {
            float v = modules[x].getCellVoltage(c);
            if (v > 0.1f) {  // ignore unpopulated / not-yet-received
                if (v > settings.OverVSetpoint)  { f |= 0x01; }
                if (v < settings.UnderVSetpoint) { f |= 0x02; }
            }
        }
        // OT/UT
        float hi = modules[x].getHighTemp();
        float lo = modules[x].getLowTemp();
        if (hi > settings.OverTSetpoint)  { a |= 0x01; }
        if (lo < settings.UnderTSetpoint) { a |= 0x02; }
        modules[x].setFaults(f);
        modules[x].setAlerts(a);
        if (f || a) isFaulted = true;
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
float BMSModuleManager::getPackVoltage()    const { return packVolt; }
float BMSModuleManager::getLowCellVolt()    const { return (lowCellVolt < 9999.0f) ? lowCellVolt : 0.0f; }
float BMSModuleManager::getHighCellVolt()   const { return highCellVolt; }
int   BMSModuleManager::getNumModules()     const { return numFoundModules; }
bool  BMSModuleManager::isFaultedState()    const { return isFaulted; }
bool  BMSModuleManager::getBalanceInhibit() const { return balanceInhibit; }
void  BMSModuleManager::setBalanceInhibit(bool inhibit) { balanceInhibit = inhibit; }
void  BMSModuleManager::setBatteryID(int id)  { batteryID = id; }
void  BMSModuleManager::setPstrings(int p)    { Pstring = (p > 0) ? p : 1; }

BMSModule& BMSModuleManager::getModule(int addr)
{
    if (addr < 0 || addr > MAX_MODULE_ADDR) return modules[0];
    return modules[addr];
}

void BMSModuleManager::setSensors(int sensor, float ignoreV)
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++) {
        modules[x].settempsensor(sensor);
        modules[x].setIgnoreCell(ignoreV);
    }
}

float BMSModuleManager::getAvgTemperature() const
{
    float sum = 0.0f;
    int   cnt = 0;
    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        if (modules[x].isExisting()) {
            float t = modules[x].getAvgTemp();
            if (t > -70.0f) { sum += t; cnt++; }
        }
    }
    return (cnt > 0) ? sum / cnt : 0.0f;
}

float BMSModuleManager::getAvgCellVolt() const
{
    float sum = 0.0f;
    int   cnt = 0;
    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        if (modules[x].isExisting()) { sum += modules[x].getAverageV(); cnt++; }
    }
    return (cnt > 0) ? sum / cnt : 0.0f;
}

float BMSModuleManager::getSoCPercent() const
{
    float lo  = settings.socLo;
    float hi  = settings.socHi;
    int   ns  = (settings.numSeries > 0) ? settings.numSeries : BMS_NUM_SERIES;
    if (hi <= lo || ns == 0) return 0.0f;
    float v = packVolt / (float)ns;
    float soc = (v - lo) * 100.0f / (hi - lo);
    if (soc > 100.0f) soc = 100.0f;
    if (soc < 0.0f)   soc = 0.0f;
    return soc;
}

// ---------------------------------------------------------------------------
// printPackSummary / printPackDetails
// ---------------------------------------------------------------------------
void BMSModuleManager::printPackSummary()
{
    Logger::console("================ BMW i3 Pack Status ================");
    Logger::console(isFaulted ? "  *** FAULTED ***" : "  All systems go!");
    Logger::console("Modules:%d  Pack:%.2fV  AvgCell:%.3fV  AvgTemp:%.1fC  SoC:%.0f%%",
                    numFoundModules, packVolt, getAvgCellVolt(),
                    getAvgTemperature(), getSoCPercent());
    Logger::console("CellLo:%.3fV  CellHi:%.3fV  Delta:%.0fmV",
                    getLowCellVolt(), highCellVolt,
                    (highCellVolt - getLowCellVolt()) * 1000.0f);
    Logger::console("");
    for (int y = 1; y <= BMW_I3_MAX_MODS; y++) {
        if (!modules[y].isExisting()) continue;
        uint8_t f = modules[y].getFaults();
        uint8_t a = modules[y].getAlerts();
        Logger::console("Mod #%d  %.2fV  lo:%.3fV hi:%.3fV  T:%.1f/%.1fC%s%s",
                        y,
                        modules[y].getModuleVoltage(),
                        modules[y].getLowCellV(), modules[y].getHighCellV(),
                        modules[y].getTemperature(0), modules[y].getTemperature(1),
                        f ? "  [FAULT]" : "",
                        a ? "  [ALERT]" : "");
    }
    Logger::console("=====================================================");
}

void BMSModuleManager::printPackDetails()
{
    Logger::console("================ BMW i3 Pack Details ================");
    Logger::console(isFaulted ? "  *** FAULTED ***" : "  All systems go!");
    for (int y = 1; y <= BMW_I3_MAX_MODS; y++) {
        if (!modules[y].isExisting()) continue;
        SERIALCONSOLE.printf("Mod #%d  %.3fV", y, modules[y].getModuleVoltage());
        for (int c = 0; c < BMW_I3_CELLS_PER_MOD; c++) {
            SERIALCONSOLE.printf("  C%02d:%.3fV", c + 1, modules[y].getCellVoltage(c));
            if (c == 5) SERIALCONSOLE.print("\n       ");
        }
        SERIALCONSOLE.printf("  T1:%.1fC T2:%.1fC\n",
                             modules[y].getTemperature(0), modules[y].getTemperature(1));
    }
    Logger::console("=====================================================");
}
