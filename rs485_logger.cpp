#include "rs485_logger.h"

bool enableRS485Logging = true; // Set false for production!

char rs485LogBuffer[RS485_LOG_BUFFER_SIZE];
size_t rs485LogHead = 0;

void rs485LogByte(uint8_t b, size_t idx, unsigned long ts) {
    // Format: [timestamp ms] idx: 0xXX
    char temp[64];
    int len = snprintf(temp, sizeof(temp),
        "[%8lu ms] idx %04u: 0x%02X (%c)\n", ts, (unsigned)idx, b, (b >= 32 && b <= 126) ? b : '.');
    // Write to Serial if desired
    if (enableRS485Logging) Serial.write((const uint8_t*)temp, len);
    // Write to circular buffer
    for (int i = 0; i < len; i++) {
        rs485LogBuffer[rs485LogHead] = temp[i];
        rs485LogHead = (rs485LogHead + 1) % RS485_LOG_BUFFER_SIZE;
    }
}

void rs485LogPrint(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char temp[120];
    int len = vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);
    if (enableRS485Logging) Serial.write((const uint8_t*)temp, len);
    for (int i = 0; i < len; i++) {
        rs485LogBuffer[rs485LogHead] = temp[i];
        rs485LogHead = (rs485LogHead + 1) % RS485_LOG_BUFFER_SIZE;
    }
}

String getRS485Logs() {
    String logs;
    for (size_t i = 0; i < RS485_LOG_BUFFER_SIZE; i++) {
        logs += rs485LogBuffer[(rs485LogHead + i) % RS485_LOG_BUFFER_SIZE];
    }
    return logs;
}