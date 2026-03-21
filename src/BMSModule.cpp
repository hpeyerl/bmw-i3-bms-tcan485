// =============================================================================
// BMSModule.cpp - BMS module data model (BMW i3 CSC variant)
// Data is written by BMSModuleManager::getAllVoltTempFromCAN() which reads
// from the CANManager staging buffers. No UART, no register access here.
// =============================================================================
#include "BMSModule.h"
#include "bms_config.h"
#include <string.h>

BMSModule::BMSModule()
{
    memset(cellVolt, 0, sizeof(cellVolt));
    moduleVolt      = 0.0f;
    temperatures[0] = 0.0f;
    temperatures[1] = 0.0f;
    ignoreCell      = 0.0f;
    exists          = false;
    faults          = 0;
    alerts          = 0;
    sensorSel       = 0;
    moduleAddress   = 0;
    numCells        = BMW_I3_CELLS_PER_MOD;
}

// ---------------------------------------------------------------------------
// Setters (written by BMSModuleManager from CAN staging data)
// ---------------------------------------------------------------------------
void BMSModule::setAddress(int addr)    { moduleAddress = (uint8_t)addr; }
void BMSModule::setExists(bool ex)      { exists = ex; }
void BMSModule::setNumCells(int n)      { numCells = (n > 0 && n <= 12) ? n : 12; }
void BMSModule::setIgnoreCell(float t)  { ignoreCell = t; }
void BMSModule::settempsensor(int s)    { sensorSel = s; }
void BMSModule::setFaults(uint8_t f)    { faults = f; }
void BMSModule::setAlerts(uint8_t a)    { alerts = a; }
void BMSModule::setModuleVoltage(float v) { moduleVolt = v; }

void BMSModule::setCellVoltage(int cell, float v)
{
    if (cell >= 0 && cell < 12) cellVolt[cell] = v;
}

void BMSModule::setTemperature(int idx, float t)
{
    if (idx == 0 || idx == 1) temperatures[idx] = t;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------
int     BMSModule::getAddress()  const { return moduleAddress; }
bool    BMSModule::isExisting()  const { return exists; }
int     BMSModule::getNumCells() const { return numCells; }
uint8_t BMSModule::getFaults()        const { return faults; }
uint8_t BMSModule::getAlerts()        const { return alerts; }
float   BMSModule::getModuleVoltage() const { return moduleVolt; }

float BMSModule::getCellVoltage(int cell) const
{
    if (cell < 0 || cell >= numCells) return 0.0f;
    return cellVolt[cell];
}

float BMSModule::getTemperature(int idx) const
{
    if (idx < 0 || idx > 1) return 0.0f;
    return temperatures[idx];
}

float BMSModule::getLowCellV() const
{
    float lo = 9999.0f;
    for (int i = 0; i < numCells; i++) {
        if (cellVolt[i] > ignoreCell && cellVolt[i] < lo)
            lo = cellVolt[i];
    }
    return (lo < 9999.0f) ? lo : 0.0f;
}

float BMSModule::getHighCellV() const
{
    float hi = -9999.0f;
    for (int i = 0; i < numCells; i++) {
        if (cellVolt[i] > ignoreCell && cellVolt[i] > hi)
            hi = cellVolt[i];
    }
    return (hi > -9999.0f) ? hi : 0.0f;
}

float BMSModule::getAverageV() const
{
    float sum = 0.0f;
    int   cnt = 0;
    for (int i = 0; i < numCells; i++) {
        if (cellVolt[i] > ignoreCell) { sum += cellVolt[i]; cnt++; }
    }
    return (cnt > 0) ? sum / cnt : 0.0f;
}

float BMSModule::getLowTemp() const
{
    if (sensorSel == 1) return temperatures[0];
    if (sensorSel == 2) return temperatures[1];
    return (temperatures[0] < temperatures[1]) ? temperatures[0] : temperatures[1];
}

float BMSModule::getHighTemp() const
{
    if (sensorSel == 1) return temperatures[0];
    if (sensorSel == 2) return temperatures[1];
    return (temperatures[0] > temperatures[1]) ? temperatures[0] : temperatures[1];
}

float BMSModule::getAvgTemp() const
{
    if (sensorSel == 1) return temperatures[0];
    if (sensorSel == 2) return temperatures[1];
    return (temperatures[0] + temperatures[1]) * 0.5f;
}
