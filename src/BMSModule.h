#pragma once
// =============================================================================
// BMSModule.h - BMS module data model (BMW i3 CSC variant)
// Pure data model: no hardware dependencies, no Tesla UART code.
// Supports up to 12 cells per module (BMW i3 CSC = 12 cells).
// =============================================================================
#include <stdint.h>
#include "bms_config.h"

class BMSModule
{
public:
    BMSModule();

    // Voltage getters
    float getCellVoltage(int cell)  const;
    float getLowCellV()             const;
    float getHighCellV()            const;
    float getAverageV()             const;
    float getModuleVoltage()        const;

    // Temperature getters
    float getTemperature(int idx)   const;
    float getLowTemp()              const;
    float getHighTemp()             const;
    float getAvgTemp()              const;

    // Fault / alert
    uint8_t getFaults()             const;
    uint8_t getAlerts()             const;

    // Address and existence
    void setAddress(int newAddr);
    int  getAddress() const;
    bool isExisting() const;
    void setExists(bool ex);

    // Cell count
    void setNumCells(int n);
    int  getNumCells() const;

    // Direct-write setters used by CAN-sourced path (BMW i3)
    void setCellVoltage(int cell, float v);
    void setTemperature(int idx, float t);
    void setModuleVoltage(float v);
    void setFaults(uint8_t f);
    void setAlerts(uint8_t a);

    // Ignore threshold (cells below this V are excluded from min/avg)
    void setIgnoreCell(float thresh);

    // Temperature sensor selection (0=avg both, 1=TS1 only, 2=TS2 only)
    void settempsensor(int sensor);

private:
    float   cellVolt[12];
    float   moduleVolt;
    float   temperatures[2];
    float   ignoreCell;    // cells below this voltage excluded from getLowCellV()
    bool    exists;
    uint8_t faults;
    uint8_t alerts;
    int     sensorSel;     // 0=avg, 1=TS1, 2=TS2
    uint8_t moduleAddress;
    uint8_t numCells;      // active cell count for this module (typically 12)
};
