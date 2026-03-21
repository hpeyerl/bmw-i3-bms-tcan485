#pragma once
// =============================================================================
// SerialConsole.h - USB serial debug interface for BMW i3 / T-CAN485
// =============================================================================
#include <Arduino.h>

class SerialConsole {
public:
    SerialConsole();
    void init();
    void loop();
    void printMenu();

private:
    void serialEvent();
    void handleConsoleCmd();
    void handleShortCmd();
    void handleConfigCmd();

    unsigned char cmdBuffer[82];
    int           ptrBuffer;
    int           state;
    int           loopcount;
    bool          cancel;

    static const int STATE_ROOT_MENU = 0;
};
