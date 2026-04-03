#pragma once

#include <Arduino.h>

// Enable/disable RS485 logging. Disable for production!
extern bool enableRS485Logging;

// Size of log buffer (increase if needed)
#define RS485_LOG_BUFFER_SIZE 8192

// Main log buffer
extern char rs485LogBuffer[RS485_LOG_BUFFER_SIZE];
extern size_t rs485LogHead;

// Log a byte with its index and timestamp
void rs485LogByte(uint8_t b, size_t idx, unsigned long ts);

// Macro to log RS485 events (disable in production)
#define DEBUG_RS485_LOG(...) do { if (enableRS485Logging) rs485LogPrint(__VA_ARGS__); } while (0)

// Print formatted log message to buffer (and Serial if enabled)
void rs485LogPrint(const char *fmt, ...);

// Utility to get all logs (for web view)
String getRS485Logs();