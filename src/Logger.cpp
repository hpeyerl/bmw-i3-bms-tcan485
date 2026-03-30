/*
   Logger.cpp

  Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

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
// Logger.cpp
// FIX: All format string arguments changed from char* to const char*
// =============================================================================
#include "Logger.h"

Logger::LogLevel Logger::logLevel    = Logger::Info;
uint32_t         Logger::lastLogTime = 0;

void Logger::debug(const char *message, ...) {
    if (logLevel > Debug) return;
    va_list args; va_start(args, message);
    Logger::log(Debug, message, args); va_end(args);
}
void Logger::info(const char *message, ...) {
    if (logLevel > Info) return;
    va_list args; va_start(args, message);
    Logger::log(Info, message, args); va_end(args);
}
void Logger::warn(const char *message, ...) {
    if (logLevel > Warn) return;
    va_list args; va_start(args, message);
    Logger::log(Warn, message, args); va_end(args);
}
void Logger::error(const char *message, ...) {
    if (logLevel > Error) return;
    va_list args; va_start(args, message);
    Logger::log(Error, message, args); va_end(args);
}
void Logger::console(const char *message, ...) {
    va_list args; va_start(args, message);
    Logger::logMessage(message, args); va_end(args);
}

void Logger::setLoglevel(LogLevel level) { logLevel = level; }
Logger::LogLevel Logger::getLogLevel()   { return logLevel;  }
uint32_t Logger::getLastLogTime()        { return lastLogTime; }
boolean  Logger::isDebug()               { return logLevel == Debug; }

void Logger::log(LogLevel level, const char *format, va_list args) {
    lastLogTime = millis();
    SERIALCONSOLE.print(lastLogTime);
    SERIALCONSOLE.print(" - ");
    switch (level) {
        case Debug:  SERIALCONSOLE.print("DEBUG");   break;
        case Info:   SERIALCONSOLE.print("INFO");    break;
        case Warn:   SERIALCONSOLE.print("WARNING"); break;
        case Error:  SERIALCONSOLE.print("ERROR");   break;
        default: break;
    }
    SERIALCONSOLE.print(": ");
    logMessage(format, args);
}

void Logger::logMessage(const char *format, va_list args) {
    // Use vsnprintf so all standard qualifiers work: %.2f, %02X, %d, %s etc.
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    SERIALCONSOLE.println(buf);
}
