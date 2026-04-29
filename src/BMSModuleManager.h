#pragma once
// =============================================================================
// BMSModuleManager.h - Pack-level BMS manager (BMW i3 CAN only)
// =============================================================================
#include "bms_config.h"
#include "BMSModule.h"
#include "CANManager.h"

class BMSModuleManager
{
public:
    BMSModuleManager();

    // Data acquisition from CAN staging buffers
    void getAllVoltTempFromCAN();

    // Fault detection (called after getAllVoltTempFromCAN)
    void checkFaults();

    // Pack-level data
    float   getPackVoltage()      const;
    float   getAvgTemperature()   const;
    float   getHighTemperature()   const;
    float   getLowTemperature()   const;
    float   getAvgCellVolt()      const;
    float   getLowCellVolt()      const;
    float   getHighCellVolt()     const;
    int     getNumModules()       const;
    bool    isFaultedState()      const;

    // Per-module accessor (for WiFi/console output)
    BMSModule& getModule(int addr);

    // Configuration
    void setBatteryID(int id);
    void setPstrings(int p);
    void setSensors(int sensor, float ignoreV);

    // Balance inhibit (drive mode or CAN charger heartbeat timeout)
    void setBalanceInhibit(bool inhibit);
    bool getBalanceInhibit()      const;

    // Serial console debug output
    void printPackSummary();
    void printPackDetails();

    // SOC (saved to NVS periodically as no LVD ISR on ESP32)
    float getSoCPercent() const;

private:
    BMSModule modules[MAX_MODULE_ADDR + 1];
    float     packVolt;
    float     lowCellVolt;
    float     highCellVolt;
    float     lowestPackVolt;
    float     highestPackVolt;
    int       numFoundModules;
    int       Pstring;
    int       batteryID;
    bool      isFaulted;
    bool      balanceInhibit;
};
