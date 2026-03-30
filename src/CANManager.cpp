// =============================================================================
// CANManager.cpp  - BMW i3 CSC BMS on LilyGo T-CAN485
// =============================================================================
#include "CANManager.h"
#include "bms_config.h"
#include "pin_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include "WiFiManager.h"
#include "CRC8.h"

extern BMSModuleManager bms;
extern EEPROMSettings   settings;

CANManager::CANManager()
    : running(false), rxTaskHandle(nullptr),
      lastChargerSeen(0), canCurrentA(0.0f),
      miniE_nextmes(0), miniE_mescycle(0), miniE_testcycle(0),
      bmwI3Bus_counter(0),
      unassignedSeen(false)
{
    memset(i3data, 0, sizeof(i3data));
    memset(i3acc,  0, sizeof(i3acc));
    memset(unassignedDMC, 0, sizeof(unassignedDMC));
}

bool CANManager::begin()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Logger::error("CAN: twai_driver_install failed (GPIO%d/GPIO%d)",
                      PIN_CAN_TX, PIN_CAN_RX);
        return false;
    }
    if (twai_start() != ESP_OK) {
        Logger::error("CAN: twai_start failed");
        twai_driver_uninstall();
        return false;
    }
    // ESP32 chip V3 TWAI IER workaround - uncomment if CAN RX is silent:
    // *((volatile uint32_t *)0x3FF6B014) = 0xEF;

    running = true;
    xTaskCreatePinnedToCore(rxTaskFn, "CAN_RX", 4096, this, 5, &rxTaskHandle, 0);
    Logger::info("CAN started: TX=GPIO%d RX=GPIO%d 500kbps", PIN_CAN_TX, PIN_CAN_RX);
    sendI3WakeFrame();
    return true;
}

void CANManager::end()
{
    if (!running) return;
    running = false;
    if (rxTaskHandle) { vTaskDelete(rxTaskHandle); rxTaskHandle = nullptr; }
    twai_stop();
    twai_driver_uninstall();
    Logger::info("CAN stopped");
}

bool CANManager::isRunning() const { return running; }

void CANManager::rxTaskFn(void *param)
{
    CANManager *self = (CANManager *)param;
    twai_message_t msg;
    while (self->running) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK)
            self->processRxFrame(msg);
    }
    vTaskDelete(nullptr);
}

void CANManager::processRxFrame(const twai_message_t &msg)
{
    uint32_t id  = msg.identifier;
    uint8_t  dlc = msg.data_length_code;

    // Charger heartbeat
    if (id == settings.chargerHeartbeatID || id == 0x305 || id == 0x306) {
        lastChargerSeen = millis();
        if ((id == 0x305 || id == 0x306) && dlc >= 2) {
            int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
            canCurrentA = raw * 0.1f;
        }
    }

    // BMW i3 cell voltage frames 0x3D1..0x3D8
    if (id >= BMW_I3_CELL_BASE && id <= (BMW_I3_CELL_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_CELL_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS) return;
        uint8_t sf = (dlc >= 8) ? (msg.data[7] & 0x03) : 0;
        int     base = sf * 4;
        for (int c = 0; c < 4 && (base + c) < 12; c++) {
            uint16_t raw = ((uint16_t)msg.data[c * 2] << 8) | msg.data[c * 2 + 1];
            i3acc[mod].cells[base + c] = raw * 0.001f;
        }
        i3acc[mod].framesRx |= (1 << sf);
        if (i3acc[mod].framesRx == 0x07) {
            memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
            i3data[mod].fresh      = true;
            i3data[mod].lastSeenMs = millis();
            i3acc[mod].framesRx    = 0;
        }
        return;
    }

    // BMW i3 temperature frames 0x3B1..0x3B8
    if (id >= BMW_I3_TEMP_BASE && id <= (BMW_I3_TEMP_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_TEMP_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 4) return;
        int16_t t1 = (int16_t)((msg.data[0] << 8) | msg.data[1]);
        int16_t t2 = (int16_t)((msg.data[2] << 8) | msg.data[3]);
        i3data[mod].temp[0]    = t1 * 0.1f;
        i3data[mod].temp[1]    = t2 * 0.1f;
        i3data[mod].lastSeenMs = millis();
        return;
    }

    // BMW i3 enumeration reply 0x4A0
    if (id == 0x4A0 && dlc >= 8) {
        memcpy(unassignedDMC, msg.data, 8);
        unassignedSeen = true;
        Logger::info("CAN: unassigned CSC DMC: %02X %02X %02X %02X %02X %02X %02X %02X",
                     msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                     msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        return;
    }

    // ==========================================================================
    // BMW i3 CSC (BMWI3BUS variant) - confirmed protocol from SME capture
    //
    // Frame ID structure: upper byte = type, lower nibble = module address (0-based)
    //   0x10N = CSC heartbeat/status (N = module addr)
    //   0x11N = CSC init/ack
    //   0x12N = cells 1-3   (LE 16-bit, 1mV/bit, D7=0, D8=CRC)
    //   0x13N = cells 4-6
    //   0x14N = cells 7-9
    //   0x15N = cells 10-12
    //   0x16N = raw NTC ADC values (3x LE 16-bit thermistors, D7=0, D8=CRC)
    //   0x17N = decoded status2 (D5 = temperature + 40 = degC)
    //   0x1CN = balance/fault status flags
    //   0x1DN = additional status flags
    // ==========================================================================
    if (settings.CSCvariant == CSC_VARIANT_BMWI3BUS &&
        id >= BMWI3BUS_CELL_BASE && id <= 0x1FF)
    {
        int mod_addr = (int)(id & 0x00F);         // lower nibble = module address
        int type     = (int)(id & 0x1F0) >> 4;   // bits[7:4] = frame type
        int mod      = mod_addr + 1;              // 1-based slot
        if (mod < 1 || mod >= I3_MAX_MODS) return;

        // Cell voltage frames: type 0x12-0x15, 3 cells each, LE 16-bit, 1mV/bit
        if (type >= 0x12 && type <= 0x15 && dlc >= 6) {
            int sub  = type - 0x12;  // 0=cells1-3, 1=cells4-6, 2=cells7-9, 3=cells10-12
            int base = sub * 3;
            for (int c = 0; c < 3; c++) {
                uint16_t raw = (uint16_t)msg.data[c * 2] |
                               ((uint16_t)msg.data[c * 2 + 1] << 8);
                if (raw > 0 && raw < 5000) {
                    i3acc[mod].cells[base + c] = raw * 0.001f;
                }
            }
            i3acc[mod].framesRx |= (1 << sub);
            if (i3acc[mod].framesRx == 0x0F) {
                memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
                i3data[mod].fresh      = true;
                i3data[mod].lastSeenMs = millis();
                i3acc[mod].framesRx    = 0;
                Logger::debug("CAN RX: mod %d cells updated (%.3fV avg)",
                              mod, (i3data[mod].cellV[0] + i3data[mod].cellV[11]) / 2.0f);
            }
            return;
        }

        // Decoded temperature frame: type 0x17, D5 = temp + 40 (degC)
        if (type == 0x17 && dlc >= 5) {
            i3data[mod].temp[0]    = (float)msg.data[4] - 40.0f;
            i3data[mod].temp[1]    = i3data[mod].temp[0];
            i3data[mod].lastSeenMs = millis();
            return;
        }

        // Raw NTC ADC frame: type 0x16 — store for diagnostics
        if (type == 0x16 && dlc >= 6) {
            memcpy(i3data[mod].dmcBytes, msg.data, 6);
            return;
        }

        // Heartbeat: type 0x10 — keep module alive timestamp
        if (type == 0x10) {
            i3data[mod].lastSeenMs = millis();
            return;
        }

        return;
    }

    // Mini-E cell voltage frames: 0x0A0-0x15F
    // Lower nibble = module (1-based), upper nibble = sub-frame type
    //   0x0N0 = error/balance status
    //   0x0N2 = cells 0-2
    //   0x0N3 = cells 3-5
    //   0x0N4 = cells 6-8
    //   0x0N5 = cells 9-11
    if (settings.CSCvariant == CSC_VARIANT_MINIE &&
        id >= MINIE_CELL_BASE && id <= MINIE_CELL_MAX)
    {
        int mod_addr = (int)(id & 0x00F);          // module number 1..8
        int type     = (int)((id & 0x0F0)>>4);     // sub-frame type
        int mod      = mod_addr + 1;
        int sub = -1;

        if (mod < 1 || mod >= I3_MAX_MODS) return;

        if (type < 6 && type > 1)
            sub = type-2;    // 0x20>>4-2 = 0

        if (sub >= 0 && dlc >= 6) {
            int base = sub * 3;
            for (int c = 0; c < 3; c++) {
                uint8_t lo = msg.data[c * 2];
                uint8_t hi = msg.data[c * 2 + 1];
                if (hi < 0x40) {
                    i3acc[mod].cells[base + c] = float(lo + (hi & 0x3F) * 256) / 1000.0f;
                }
            }
            i3acc[mod].framesRx |= (1 << sub);
            if (i3acc[mod].framesRx == 0x0F) {
                memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
                i3data[mod].fresh      = true;
                i3data[mod].lastSeenMs = millis();
                i3acc[mod].framesRx    = 0;
            }
        }
        return;
    }

    // Mini-E temperature frames: 0x170..0x17F
    if (settings.CSCvariant == CSC_VARIANT_MINIE &&
        id >= MINIE_TEMP_BASE && id <= MINIE_TEMP_MAX)
    {
        int mod = (int)(id & 0x00F) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 2) return;
        i3data[mod].temp[0]    = (float)msg.data[0] - 40.0f;
        i3data[mod].temp[1]    = (float)msg.data[1] - 40.0f;
        i3data[mod].lastSeenMs = millis();
        return;
    }

    wifiLogCAN(id, (uint8_t *)msg.data, dlc);
}

bool  CANManager::getChargerActive() const {
    if (lastChargerSeen == 0) return false;
    return (millis() - lastChargerSeen) < CHARGER_TIMEOUT_MS;
}
float CANManager::getCanCurrentA() const { return canCurrentA; }

bool CANManager::getI3SlaveData(int addr, I3SlaveData &out)
{
    if (addr < 1 || addr >= I3_MAX_MODS) return false;
    out = i3data[addr];
    i3data[addr].fresh = false;
    return out.lastSeenMs > 0;
}

bool           CANManager::hasUnassignedCSC()    const { return unassignedSeen; }
void           CANManager::clearUnassignedFlag()       { unassignedSeen = false; }
const uint8_t* CANManager::getUnassignedDMC()    const { return unassignedDMC; }

void CANManager::sendI3WakeFrame()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_WAKE_ID, data, 8);
    Logger::info("CAN: i3 wake frame sent (0x%03X)", BMW_I3_WAKE_ID);
}

void CANManager::sendI3FindUnassigned()
{
    uint8_t data[8] = {0x37, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    sendFrame(BMW_I3_CMD_ID, data, 8);
    Logger::debug("CAN: i3 find-unassigned sent");
}

void CANManager::sendI3AssignID(uint8_t newID, const uint8_t dmcBytes[8])
{
    uint8_t data[8];
    data[0] = 0x12; data[1] = 0xAB;
    memcpy(&data[2], dmcBytes, 4);
    data[6] = 0xFF; data[7] = 0xFF;
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(30);

    data[1] = 0xBA;
    memcpy(&data[2], dmcBytes + 4, 4);
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(10);

    memset(data, 0xFF, 8);
    data[0] = 0x5B; data[1] = newID;
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(10);

    memset(data, 0xFF, 8);
    data[0] = 0x37; data[1] = newID;
    sendFrame(BMW_I3_CMD_ID, data, 8);
    Logger::info("CAN: i3 assigned ID %d", newID);
}

void CANManager::sendI3ResetAllIDs(uint8_t maxID)
{
    uint8_t data[8] = {0xA1, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (uint8_t id = 0; id <= maxID; id++) {
        data[1] = id;
        sendFrame(BMW_I3_CMD_ID, data, 8);
        delay(2);
    }
    Logger::info("CAN: i3 ID reset broadcast (0..%d)", maxID);
}

void CANManager::sendI3BalanceReset()
{
    uint8_t data[8] = {0xFF, 0x00, 0xCD, 0xA2, 0x00, 0x00, 0x00, 0x00};
    sendFrame(BMW_I3_BAL_RESET_ID, data, 8);
    Logger::info("CAN: i3 balance reset sent");
}

// =============================================================================
// Mini-E support (CSC_VARIANT_MINIE)
// =============================================================================

// CRC8 finalxor table from reference implementation (Tom-evnut/BMWPhevBMS)
static const uint8_t miniE_finalxor[12] = {
    0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C
};

uint8_t CANManager::miniEChecksum(uint32_t msgId, const uint8_t *buf, uint8_t len, uint8_t idx)
{
    static CRC8 crc8;
    static bool crc8_init = false;
    if (!crc8_init) { crc8.begin(); crc8_init = true; }
    uint8_t canmes[11];
    canmes[0] = (msgId >> 8) & 0xFF;
    canmes[1] =  msgId       & 0xFF;
    int meslen = len + 1;
    for (int i = 0; i < len - 1; i++) canmes[i + 2] = buf[i];
    return crc8.get_crc8(canmes, meslen, miniE_finalxor[idx % 12]);
}

void CANManager::sendMiniECommand()
{
    if (!running) return;

    if (miniE_mescycle > 0x0F) miniE_mescycle = 0;

    if (miniE_nextmes >= 0x0C) {
        miniE_nextmes = 0;
        if (miniE_testcycle < 4) miniE_testcycle++;
    }

    uint32_t msgId = MINIE_CMD_BASE | miniE_nextmes;
    uint8_t buf[8] = {0};

    buf[0] = 0x68; buf[1] = 0x10;
    buf[2] = 0x00;

    if (miniE_testcycle < 3) {
        buf[3] = 0x00; buf[4] = 0x00;
    } else {
        buf[3] = 0x50;
        buf[4] = 0x00;
    }
    buf[5] = 0x00;
    buf[6] = miniE_mescycle << 4;
    if (miniE_testcycle == 2) buf[6] |= 0x04;
    buf[7] = miniEChecksum(msgId, buf, 8, miniE_nextmes);

    sendFrame(msgId, buf, 8);

    miniE_mescycle++;
    miniE_nextmes++;
}

// =============================================================================
// BMWI3BUS TX command (CSC_VARIANT_BMWI3BUS)
//
// Confirmed SME TX format from capture:
//   IDs:  0x080-0x087 (one per CSC slot)
//   Data: C7 10 00 50 20 00 [counter] [CRC]
//   Counter increments by 0x10 per cycle, wraps at 0x100
//   CRC: same CRC8 finalxor as Mini-E, indexed by slot
// =============================================================================
// =============================================================================
// BMWI3BUS TX command (CSC_VARIANT_BMWI3BUS)
//
// Confirmed SME init sequence from capture (frames 0-44 before first cell data):
//   Cycle 0: D4=0x00, counter=0x10
//   Cycle 1: D4=0x00, counter=0x20
//   Cycle 2: D4=0x00, counter=0x34  (note: not 0x30 - SME skips here)
//   Cycle 3: D4=0x10, counter=0x40  (CSC 0x0115 D1 goes 0x10 after this)
//   Cycle 4+: D4=0x50, counter increments 0x10 per cycle (steady state)
// =============================================================================
void CANManager::sendBMWI3BUSCommand()
{
    if (!running) return;

    uint8_t d4;
    uint8_t counter;

    // Init sequence: 4 special cycles before steady state
    static const uint8_t init_d4[4]      = { 0x00, 0x00, 0x00, 0x10 };
    static const uint8_t init_counter[4] = { 0x10, 0x20, 0x34, 0x40 };

    if (bmwI3Bus_counter < 4) {
        // Still in init sequence
        d4      = init_d4[bmwI3Bus_counter];
        counter = init_counter[bmwI3Bus_counter];
    } else {
        // Steady state: D4=0x50, counter runs 0x50, 0x60, ... wrapping
        d4      = 0x50;
        counter = 0x40 + ((bmwI3Bus_counter - 3) * 0x10) & 0xFF;
    }

    for (uint8_t slot = 0; slot < 8; slot++) {
        uint32_t msgId = MINIE_CMD_BASE | slot;
        uint8_t buf[8];
        buf[0] = 0xC7;
        buf[1] = 0x10;
        buf[2] = 0x00;
        buf[3] = d4;
        buf[4] = 0x20;
        buf[5] = 0x00;
        buf[6] = counter;
        buf[7] = miniEChecksum(msgId, buf, 8, slot);
        sendFrame(msgId, buf, 8);
    }

    bmwI3Bus_counter++;
}

void CANManager::sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extended)
{
    if (!running) return;
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = id;
    msg.extd             = extended ? 1 : 0;
    msg.data_length_code = (len <= 8) ? len : 8;
    memcpy(msg.data, data, msg.data_length_code);
    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (res != ESP_OK)
        Logger::debug("CAN TX err 0x%02X on ID 0x%03X", res, id);
    wifiLogCAN(id, (uint8_t *)msg.data, msg.data_length_code);
}

void CANManager::sendBatterySummary()
{
    if (!running) return;
    float packV   = bms.getPackVoltage();
    float lowC    = bms.getLowCellVolt();
    float highC   = bms.getHighCellVolt();
    float avgT    = bms.getAvgTemperature();
    bool  faulted = bms.isFaultedState();
    int   ns      = settings.numSeries > 0 ? settings.numSeries : BMS_NUM_SERIES;
    float socRange = settings.socHi - settings.socLo;
    float socPer  = 0.0f;
    if (socRange > 0.01f && ns > 0) {
        socPer = ((packV / (float)ns) - settings.socLo) / socRange * 100.0f;
        if (socPer > 100.0f) socPer = 100.0f;
        if (socPer < 0.0f)   socPer = 0.0f;
    }
    uint8_t data[8];
    // 0x351
    int16_t cvh = (int16_t)(settings.OverVSetpoint  * ns * 10.0f);
    int16_t cvl = (int16_t)(settings.UnderVSetpoint * ns * 10.0f);
    int16_t ccl = 500, dcl = 1000;
    data[0]=cvh&0xFF; data[1]=(cvh>>8)&0xFF; data[2]=ccl&0xFF; data[3]=(ccl>>8)&0xFF;
    data[4]=dcl&0xFF; data[5]=(dcl>>8)&0xFF; data[6]=cvl&0xFF; data[7]=(cvl>>8)&0xFF;
    sendFrame(0x351, data, 8);
    // 0x355
    uint16_t soc16 = (uint16_t)socPer; uint16_t soh = 100;
    uint32_t socEx = (uint32_t)(socPer * 100.0f);
    memset(data,0,8);
    data[0]=soc16&0xFF; data[1]=(soc16>>8)&0xFF; data[2]=soh&0xFF; data[3]=(soh>>8)&0xFF;
    data[4]=socEx&0xFF; data[5]=(socEx>>8)&0xFF; data[6]=(socEx>>16)&0xFF; data[7]=(socEx>>24)&0xFF;
    sendFrame(0x355, data, 8);
    // 0x356
    int16_t pv10=(int16_t)(packV*100.0f), curr10=(int16_t)(canCurrentA*10.0f), t10=(int16_t)(avgT*10.0f);
    memset(data,0,8);
    data[0]=pv10&0xFF; data[1]=(pv10>>8)&0xFF; data[2]=curr10&0xFF; data[3]=(curr10>>8)&0xFF;
    data[4]=t10&0xFF;  data[5]=(t10>>8)&0xFF;
    sendFrame(0x356, data, 8);
    // 0x35A
    memset(data,0,8);
    if (highC > settings.OverVSetpoint  - 0.05f) data[0] |= 0x04;
    if (lowC  < settings.UnderVSetpoint + 0.05f) data[0] |= 0x08;
    if (faulted) data[1] |= 0x04;
    sendFrame(0x35A, data, 8);
    // 0x35E
    const char *mfr = "BMWI3BMS"; memset(data,0x20,8);
    for (int i=0;i<8;i++) data[i]=mfr[i];
    sendFrame(0x35E, data, 8);
    // 0x35F
    memset(data,0,8); data[0]='L'; data[1]='I'; data[4]=0x01;
    sendFrame(0x35F, data, 8);
    Logger::debug("CAN TX: packV=%.2fV SoC=%d%%", packV, (int)socPer);
}
