#pragma once
// =============================================================================
// CANManager.h  - BMW i3 CSC BMS on LilyGo T-CAN485
//
// TX (SimpBMS protocol): frames 0x351/355/356/35A/35E/35F every 1 s
// RX (FreeRTOS task on Core 0):
//   - Charger/inverter heartbeat detection -> balance inhibit
//   - BMW i3 CSC cell voltage frames  0x3D1-0x3D8 (3 sub-frames per cycle)
//   - BMW i3 CSC temperature frames   0x3B1-0x3B8
//   - BMW i3 CSC response frames      0x4A0 (enumeration replies)
//
// Hardware: SN65HVD231 CAN transceiver on T-CAN485 board
//   PIN_CAN_TX / PIN_CAN_RX defined in pin_config.h
//   (Verify GPIO assignment against schematic - see pin_config.h TODO)
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// BMW i3 staging data - populated by CAN RX task, read by BMSModuleManager
#define I3_MAX_MODS     9   // index 0 unused; 1..8 = module address
struct I3SlaveData {
    float    cellV[12];     // cell voltages in volts (0.0 = not yet received)
    float    temp[2];       // temperatures in degC
    bool     fresh;         // true = new data since last read by BMSModuleManager
    uint32_t lastSeenMs;    // millis() timestamp of last received frame
    uint8_t  dmcBytes[8];   // raw DMC bytes from 0x4A0 reply (enumeration)
};

class CANManager {
public:
    CANManager();
    bool begin();
    void end();
    bool isRunning() const;

    // TX - SimpBMS summary frames
    void sendBatterySummary();

    // TX - BMW i3 CSC enumeration commands (called from SerialConsole / main)
    void sendI3WakeFrame();
    void sendI3FindUnassigned();
    void sendI3AssignID(uint8_t newID, const uint8_t dmcBytes[8]);
    void sendI3ResetAllIDs(uint8_t maxID = 14);
    void sendI3BalanceReset();

    // TX - generic frame
    void sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extended = false);

    // RX dispatch (called internally by RX task)
    void processRxFrame(const twai_message_t &msg);

    // Charger/inverter heartbeat
    bool  getChargerActive() const;
    float getCanCurrentA()   const;

    // BMW i3 slave data (called by BMSModuleManager each BMS cycle)
    bool getI3SlaveData(int addr, I3SlaveData &out);

    // Unassigned CSC detection (set by RX task when 0x4A0 seen without known ID)
    bool  hasUnassignedCSC() const;
    void  clearUnassignedFlag();
    const uint8_t* getUnassignedDMC() const;  // 8 bytes of DMC identifier

private:
    bool         running;
    TaskHandle_t rxTaskHandle;

    uint32_t     lastChargerSeen;
    float        canCurrentA;

    I3SlaveData  i3data[I3_MAX_MODS];

    // Accumulator for multi-sub-frame i3 cell sets
    struct I3CellAcc {
        float   cells[12];
        uint8_t framesRx;   // bitmask: bits 0/1/2 = sub-frames 0/1/2 received
    } i3acc[I3_MAX_MODS];

    // Unassigned CSC enumeration state
    bool    unassignedSeen;
    uint8_t unassignedDMC[8];

    static void rxTaskFn(void *param);
};
