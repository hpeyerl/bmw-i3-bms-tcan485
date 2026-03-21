#pragma once
// =============================================================================
// WiFiManager.h - BMW i3 / T-CAN485 stripped WiFi AP + JSON status
// =============================================================================
#include <Arduino.h>

class WiFiManager {
public:
    WiFiManager();
    bool   begin();
    void   end();
    bool   isRunning() const;
    String getIP()     const;
    void   loop();      // placeholder, async server needs no polling

private:
    bool   running;
    String ipAddr;
};

// Called by CANManager::sendFrame() to log frames to the rolling CAN buffer
void wifiLogCAN(uint32_t id, uint8_t *data, uint8_t len);
