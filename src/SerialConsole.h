#pragma once
/*
 * SerialConsole.h
 *
Copyright (c) 2017 EVTV / Collin Kidder

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
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
