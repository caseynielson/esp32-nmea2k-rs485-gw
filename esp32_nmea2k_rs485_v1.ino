/*
 * esp32 NMEA 2k to rs485 gateway
 */

#include <ESP32-TWAI-CAN.hpp>
#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
//added for debug
#include "rs485_logger.h"

// Version information
#define SW_VERSION_MAJOR    1
#define SW_VERSION_MINOR    7
#define SW_VERSION_PATCH    5
#define SW_VERSION_STRING   "v1.7.5"
#define SW_BUILD_DATE       "2025-10-15"

// --- CAN Bus Definitions ---
#define CAN_TX      5
#define CAN_RX      4
#define CAN_SPEED   250  // NMEA2000 uses 250kbps

// RS485 Definitions for HW-097 module
#define RS485_RX_PIN    16    // Connect to RO pin on HW-097
#define RS485_TX_PIN    17    // Connect to DI pin on HW-097  
#define RS485_DE_RE_PIN 21    // Connect to DE and RE pins on HW-097

#define RS485_BAUDRATE  76800
#define RESPONSE_TIMEOUT 3000

// RS485 message format
#define REQUEST_LEN     4
#define RESPONSE_LEN    13
#define MSG_TYPE_REQUEST 0x04
#define MSG_CMD_GET_DEPTH 0x09

HardwareSerial RS485Serial(2);

const char *ssid = "nmea2k_rs485_gw";
const char *password = "123456789";

WebServer server(80);

// --- Debug Output Control ---
bool debugNMEA2000 = true; // Set true for verbose NMEA 2000 (CAN) debug
bool debugRS485 = true;    // Set true for verbose RS485 debug

#define DEBUG_PRINT_NMEA2000(...) do { if (debugNMEA2000) logMessage(__VA_ARGS__); } while (0)
#define DEBUG_PRINT_RS485(...) do { if (debugRS485) logMessage(__VA_ARGS__); } while (0)
//#define DEBUG_PRINT_NMEA2000(...) do { if (debugNMEA2000) Serial.printf(__VA_ARGS__); } while (0)
//#define DEBUG_PRINT_RS485(...) do { if (debugRS485) Serial.printf(__VA_ARGS__); } while (0)

#define LOG_BUFFER_SIZE 4096
char logBuffer[LOG_BUFFER_SIZE];
size_t logHead = 0;

void logMessage(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char temp[128];
    int len = vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);

    // Write to Serial
    Serial.write((uint8_t*)temp, len);

    // Write to circular buffer
    for (int i = 0; i < len; i++) {
        logBuffer[logHead] = temp[i];
        logHead = (logHead + 1) % LOG_BUFFER_SIZE;
    }
}

// --- Depth data storage --- rs485
uint16_t depth_tenths = 0; // Depth in tenths of a foot (e.g., 123 = 12.3 ft)
uint8_t toggle_bit = 0;

// --- timeouts and defaults
unsigned long nmeaDepthTimeoutMs = 5000; // Default: 5 seconds
uint16_t depth_timeout_value = 0;         // Value when timeout occurs
unsigned long lastDepthReceivedMs = 0;
bool depth_valid = false;

// --- Data Structs ---
struct CurrentDepth {
  float currentDepth;
  float currentOffset;
  uint8_t currentRange;
};
CurrentDepth currentDepth;

struct CurrentGPS {
  double latitude;
  double longitude;
  double altitude;
  bool valid;
};
CurrentGPS currentGPS;

struct CurrentCOG {
  float cog;
  float sog;
  bool valid;
};
CurrentCOG currentCOG;

struct CurrentTemp {
  float waterTemp;
  float seaTemp;
};
CurrentTemp currentTemp;

struct CurrentMagVar {
  float variation;
  bool valid;
};
CurrentMagVar currentMagVar;

// --- PGN Handler Type ---
typedef void (*PGNHandler)(const CanFrame&);

// --- PGN Decoders ---
void handleDepthPGN(const CanFrame& frame) {
    // PGN 128267 Water Depth
    if (frame.data_length_code < 7) return;
    uint8_t sid = frame.data[0];
    uint32_t depthRaw = frame.data[1] | (frame.data[2] << 8) | (frame.data[3] << 16);
    int16_t offsetRaw = frame.data[4] | (frame.data[5] << 8);
    uint8_t rangeRaw = frame.data[6];
    currentDepth.currentDepth = depthRaw * 0.01f;
    currentDepth.currentOffset = offsetRaw * 0.001f;
    currentDepth.currentRange = (rangeRaw == 255) ? 0 : rangeRaw * 10;
    DEBUG_PRINT_NMEA2000("128267 Water Depth: SID = %s; Depth = %.2f m; Offset = %.3f m; Range = %s\n",
        "Unknown", currentDepth.currentDepth, currentDepth.currentOffset,
        (rangeRaw == 255) ? "Unknown" : String(currentDepth.currentRange).c_str());

    float depth_ft = currentDepth.currentDepth * 3.28084f;
    depth_tenths = (uint16_t)(depth_ft * 10.0f + 0.5f); // tenths of a foot
    lastDepthReceivedMs = millis();
    depth_valid = true;
}

void handleGPSPGN_129025(const CanFrame& frame) {
    // PGN 129025 Position, Rapid Update
    if (frame.data_length_code < 8) return;
    int32_t lonRaw = (int32_t)(
        ((uint32_t)frame.data[0]) |
        ((uint32_t)frame.data[1] << 8) |
        ((uint32_t)frame.data[2] << 16) |
        ((uint32_t)frame.data[3] << 24));
    int32_t latRaw = (int32_t)(
        ((uint32_t)frame.data[4]) |
        ((uint32_t)frame.data[5] << 8) |
        ((uint32_t)frame.data[6] << 16) |
        ((uint32_t)frame.data[7] << 24));
    currentGPS.longitude = lonRaw * 1e-7;
    currentGPS.latitude  = latRaw * 1e-7;
    currentGPS.valid = true;
    DEBUG_PRINT_NMEA2000("129025 Position, Rapid Update:  Latitude = %.7f; Longitude = %.7f\n", currentGPS.latitude, currentGPS.longitude);
}

void handleGPSPGN_129029(const CanFrame& frame) {
    // PGN 129029 GNSS Position Data
    if (frame.data_length_code < 43) return;
    uint8_t sid = frame.data[0];
    uint16_t dateRaw = frame.data[1] | (frame.data[2] << 8);
    uint32_t timeRaw = frame.data[3] | (frame.data[4] << 8) | (frame.data[5] << 16) | (frame.data[6] << 24);
    int32_t latRaw = (int32_t)(frame.data[9] | (frame.data[10] << 8) | (frame.data[11] << 16) | (frame.data[12] << 24));
    int32_t lonRaw = (int32_t)(frame.data[13] | (frame.data[14] << 8) | (frame.data[15] << 16) | (frame.data[16] << 24));
    int32_t altRaw = (int32_t)(frame.data[17] | (frame.data[18] << 8) | (frame.data[19] << 16) | (frame.data[20] << 24));
    currentGPS.latitude = latRaw * 1e-7;
    currentGPS.longitude = lonRaw * 1e-7;
    currentGPS.altitude = altRaw * 0.001;
    currentGPS.valid = true;
    DEBUG_PRINT_NMEA2000("129029 GNSS Position Data:  SID = Unknown; Date = %u; Time = %.4f; Latitude = %.7f; Longitude = %.7f; Altitude = %.6f m\n",
        dateRaw, timeRaw / 10000.0, currentGPS.latitude, currentGPS.longitude, currentGPS.altitude);
}

void handleCOGPGN(const CanFrame& frame) {
    // PGN 129026 COG & SOG, Rapid Update
    if (frame.data_length_code < 8) return;
    uint8_t sid = frame.data[0];
    uint8_t ref = frame.data[1];
    uint16_t cogRaw = frame.data[2] | (frame.data[3] << 8);
    uint16_t sogRaw = frame.data[4] | (frame.data[5] << 8);
    currentCOG.cog = cogRaw * 0.0001f * 180.0f/3.14159265358979323846f; // radians to degrees
    currentCOG.sog = sogRaw * 0.01f; // m/s
    currentCOG.valid = true;
    DEBUG_PRINT_NMEA2000("129026 COG & SOG, Rapid Update:  SID = Unknown; COG Reference = %s; COG = %.1f deg; SOG = %.2f m/s\n",
        (ref ? "True" : "False"), currentCOG.cog, currentCOG.sog);
}

void handleMagVarPGN(const CanFrame& frame) {
    // PGN 127258 Magnetic Variation
    if (frame.data_length_code < 6) return;
    uint8_t sid = frame.data[0];
    uint8_t src = frame.data[1];
    int16_t varRaw = frame.data[4] | (frame.data[5] << 8);
    currentMagVar.variation = varRaw * 0.0001 * 180.0/3.14159265358979323846;
    currentMagVar.valid = true;
    DEBUG_PRINT_NMEA2000("127258 Magnetic Variation:  SID = Unknown; Source = %s; Variation = %.1f deg\n",
        "Automatic Table", currentMagVar.variation);
}

void handleGNSSSatsInViewPGN(const CanFrame& frame) {
    // PGN 129540 GNSS Sats in View
    if (frame.data_length_code < 8) return;
    uint8_t sid = frame.data[0];
    uint8_t satsInView = frame.data[2];
    DEBUG_PRINT_NMEA2000("129540 GNSS Sats in View:  SID = Unknown; Sats in View = %u; ...\n", satsInView);
}

void handleGNSSDOPPGN(const CanFrame& frame) {
    // PGN 129539 GNSS DOPs
    if (frame.data_length_code < 8) return;
    uint8_t sid = frame.data[0];
    float hdop = (frame.data[4] | (frame.data[5] << 8)) * 0.01;
    float vdop = (frame.data[6] | (frame.data[7] << 8)) * 0.01;
    DEBUG_PRINT_NMEA2000("129539 GNSS DOPs:  SID = Unknown; HDOP = %.2f; VDOP = %.2f\n", hdop, vdop);
}

void handleEnvParamsPGN(const CanFrame& frame) {
    // PGN 130310 Environmental Parameters (obsolete)
    if (frame.data_length_code < 8) return;
    float waterTemp = (frame.data[2] | (frame.data[3] << 8)) * 0.01f;
    DEBUG_PRINT_NMEA2000("130310 Environmental Parameters (obsolete): Water Temperature = %.2f C\n", waterTemp);
    currentTemp.waterTemp = waterTemp;
}

void handleTempPGN(const CanFrame& frame) {
    // PGN 130312 Temperature
    if (frame.data_length_code < 8) return;
    float temp = (frame.data[2] | (frame.data[3] << 8)) * 0.01f;
    DEBUG_PRINT_NMEA2000("130312 Temperature: Actual Temperature = %.2f C\n", temp);
    currentTemp.seaTemp = temp;
}

void handleNavigationDataPGN(const CanFrame& frame) {
    // PGN 129284 Navigation Data
    DEBUG_PRINT_NMEA2000("129284 Navigation Data: ...\n");
}

void handleCrossTrackErrorPGN(const CanFrame& frame) {
    // PGN 129283 Cross Track Error
    DEBUG_PRINT_NMEA2000("129283 Cross Track Error: ...\n");
}

// --- Proprietary PGN 126720 Handler ---
void handleProprietary126720PGN(const CanFrame& frame) {
    // CANBOAT style: output Manufacturer, Industry, Data as hex
    const char *manufacturer = "Garmin";
    const char *industry = "Marine";
    if (!debugNMEA2000) return;
    Serial.print("Manufacturer Proprietary fast-packet addressed:  Manufacturer Code = ");
    Serial.print(manufacturer);
    Serial.print("; Industry Code = ");
    Serial.print(industry);
    Serial.print("; Data = ");
    for (int i = 0; i < frame.data_length_code; i++) {
        Serial.printf("%02X ", frame.data[i]);
    }
    Serial.println();
}

// --- PGN Table ---
struct PGNEntry {
    uint32_t pgn;
    PGNHandler handler;
};

PGNEntry pgnTable[] = {
    {128267, handleDepthPGN},
    {129025, handleGPSPGN_129025},
    {129029, handleGPSPGN_129029},
    {129026, handleCOGPGN},
    {127258, handleMagVarPGN},
    {129540, handleGNSSSatsInViewPGN},
    {129539, handleGNSSDOPPGN},
    {130310, handleEnvParamsPGN},
    {130312, handleTempPGN},
    {129284, handleNavigationDataPGN},
    {129283, handleCrossTrackErrorPGN},
    {126720, handleProprietary126720PGN},
};
const int pgnTableSize = sizeof(pgnTable) / sizeof(PGNEntry);

// --- RS485 State Tracking ---
unsigned long lastByteTime = 0;
const unsigned long MESSAGE_TIMEOUT = 1000;
unsigned long totalMessages = 0;
unsigned long invalidMessages = 0;
unsigned long depthRequests = 0;

// --- PGN extraction for J1939/NMEA2000 ---
uint32_t extractPGN(uint32_t canId) {
    uint8_t dp = (canId >> 24) & 0x01;
    uint8_t pf = (canId >> 16) & 0xFF;
    uint8_t ps = (canId >> 8) & 0xFF;
    if (pf < 240) {
        return (dp << 16) | (pf << 8);
    } else {
        return (dp << 16) | (pf << 8) | ps;
    }
}



// --- CAN Message Processing with Handler Table ---
void canReceiver() {
  CanFrame rxFrame;
  if (ESP32Can.readFrame(rxFrame)) {
    uint32_t pgn = extractPGN(rxFrame.identifier);
    bool handled = false;
    for (int i = 0; i < pgnTableSize; i++) {
      if (pgn == pgnTable[i].pgn) {
        if (pgnTable[i].handler) {
          pgnTable[i].handler(rxFrame);
        } else {
          DEBUG_PRINT_NMEA2000("[PGN %lu] Handler not implemented\n", pgn);
        }
        handled = true;
        break;
      }
    }
    if (!handled) {
      DEBUG_PRINT_NMEA2000("PGN %lu not handled\n", pgn);
    }
  }
}

// --- RS485 Request/Response Handling ---
// --- RS485 Robust Request/Response Handling with Recovery and Resync ---
// Debug macro for misalignment/drops
#define DEBUG_RS485_RESYNC(...) do { if (enableRS485Logging) rs485LogPrint(__VA_ARGS__); } while (0)

void readAndProcessRS485Message() {
    static uint8_t rxBuffer[164];         // Reception buffer for incoming bytes (changed from 128 to 164)
    static uint8_t bufferLen = 0;         // Number of bytes currently in buffer

    // Read all available bytes from RS485 into rxBuffer
    while (RS485Serial.available() && bufferLen < sizeof(rxBuffer)) {
        uint8_t b = RS485Serial.read();
        rs485LogByte(b, bufferLen, millis()); // Log every byte for analysis
        rxBuffer[bufferLen++] = b;
    }

    uint8_t scanPos = 0;
    int misalignments = 0;
    int drops = 0;

    // Try to extract as many valid frames as possible
    while (bufferLen - scanPos >= 4) { // Minimum plausible message length
        uint8_t lenByte = rxBuffer[scanPos];
        // Check if the length byte is plausible for your protocol
        if (lenByte < 4 || lenByte > 32) {
            DEBUG_RS485_RESYNC("[RS485][MISALIGN] idx:%u byte:0x%02X at %lu ms\n", scanPos, lenByte, millis());
            // Log the next 8 bytes for context
            DEBUG_RS485_RESYNC("[RS485][MISALIGN] context:");
            for (uint8_t i = 0; i < 8 && (scanPos + i) < bufferLen; i++) {
                rs485LogPrint(" %02X", rxBuffer[scanPos + i]);
            }
            rs485LogPrint("\n");
            misalignments++;
            scanPos++;
            continue;
        }
        // Do we have enough bytes for a full message?
        if (bufferLen - scanPos < lenByte) {
            // Wait for more bytes to arrive
            break;
        }
        // Candidate message: rxBuffer[scanPos .. scanPos+lenByte-1]
        uint8_t* msg = rxBuffer + scanPos;
        if (verifyRS485Checksum(msg, lenByte)) {
            // Valid message, process it!
            storeRS485Message(msg, lenByte);
            scanPos += lenByte; // Jump to next possible frame
        } else {
            // Log failed message
            DEBUG_RS485_RESYNC("[RS485][DROP] idx:%u len:%u, bytes:", scanPos, lenByte);
            for (uint8_t i = 0; i < lenByte; i++) {
                rs485LogPrint(" %02X", msg[i]);
            }
            rs485LogPrint(" at %lu ms\n", millis());
            drops++;
            scanPos++; // Try next byte as start
        }
    }
    // Remove processed or skipped bytes from buffer
    if (scanPos > 0) {
        if (scanPos < bufferLen) {
            memmove(rxBuffer, rxBuffer + scanPos, bufferLen - scanPos);
        }
        bufferLen -= scanPos;
    }

    // Optionally log summary for this round
    if (misalignments > 0 || drops > 0) {
        DEBUG_RS485_RESYNC("[RS485][SUMMARY] misalignments:%d drops:%d at %lu ms\n", misalignments, drops, millis());
    }
}

bool validateMessage(uint8_t* msg, uint8_t len) {
  if (msg[0] != len) {
    DEBUG_PRINT_RS485("RS485 Length mismatch: header %d, actual %d\n", msg[0], len);
    return false;
  }
  return true;
}

void storeRS485Message(uint8_t* msg, uint8_t len) {
    DEBUG_PRINT_RS485("RS485: Processing message (len: %d)\n", len);
    DEBUG_PRINT_RS485("HEX: ");
    for (int i = 0; i < len; i++) { DEBUG_PRINT_RS485("%02X ", msg[i]); }
    DEBUG_PRINT_RS485("\n");

    // Only handle depth requests: 04 09 XX XX
    if (len == 4 && msg[0] == 0x04 && msg[1] == 0x09) {
        DEBUG_PRINT_RS485("[RS485] Depth request received. Sending response.\n");
        sendDepthData();
        toggle_bit ^= 1;
    } else {
        DEBUG_PRINT_RS485("[RS485] Unknown or invalid request received.\n");
    }
}

// --- RS485 Checksum Calculation ---
uint8_t calculate_checksum(const uint8_t *data, uint8_t len) {
    uint8_t total = 0;
    for (uint8_t i = 0; i < len; i++) {
        total = (total + data[i]) & 0xFF;
    }
    return ((~total + 1) & 0xFF); // 8-bit two's complement
}

bool verifyRS485Checksum(const uint8_t* msg, uint8_t len) {
    if (len < 2) return false;
    uint8_t checksum = msg[len-1];
    uint8_t calculated = calculate_checksum(msg, len-1);
    return checksum == calculated;
}

// --- RS485 Transmit/Receive Direction ---
void enableTransmit() {
    digitalWrite(RS485_DE_RE_PIN, HIGH);
}

void enableReceive() {
    digitalWrite(RS485_DE_RE_PIN, LOW);
}

void sendDepthData() {
    uint16_t depth_to_send;
    if (!depth_valid || (millis() - lastDepthReceivedMs > nmeaDepthTimeoutMs)) {
    depth_to_send = depth_timeout_value;
    } else {
    depth_to_send = depth_tenths;
    }
    
    uint8_t LL = depth_to_send & 0xFF;
    uint8_t HH = (depth_to_send >> 8) & 0xFF;
    uint8_t response[RESPONSE_LEN] = {
        RESPONSE_LEN,        // Length
        MSG_CMD_GET_DEPTH,   // Command (0x09)
        0x14,                // Sub-command
        0xAA,                // Status/ID
        LL, HH,              // Depth bytes (tenths of a foot, low/high)
        0xFF, 0x03,          // Filler
        0xFF, 0x03,          // Filler
        toggle_bit,          // Toggle
        0x02,                // Filler
        0                    // Placeholder for checksum
    };
    response[12] = calculate_checksum(response, 12); // Set checksum
    enableTransmit();
    delayMicroseconds(200);
    RS485Serial.write(response, RESPONSE_LEN);
    RS485Serial.flush();
    delayMicroseconds(200);
    enableReceive();

    if (debugRS485) {
      Serial.print("[RS485] Response sent: ");
      for (int i = 0; i < RESPONSE_LEN; i++) {
      Serial.printf("%02X ", response[i]);
      }
      Serial.println();
    }
}

// --- Debug/Serial Command Handler ---
void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "debug nmea on") {
            debugNMEA2000 = true;
            Serial.println("NMEA2000 debug output ON");
        } else if (command == "debug nmea off") {
            debugNMEA2000 = false;
            Serial.println("NMEA2000 debug output OFF");
        } else if (command == "debug rs485 on") {
            debugRS485 = true;
            Serial.println("RS485 debug output ON");
        } else if (command == "debug rs485 off") {
            debugRS485 = false;
            Serial.println("RS485 debug output OFF");
        } else if (command == "depth") {
            Serial.printf("Current depth: %.1f ft (tenths: %u)\n", depth_tenths/10.0, depth_tenths);
        } else {
            Serial.println("Unknown command.");
        }
    }
}

void printMessageStats() {
  unsigned long total = totalMessages + invalidMessages;
  float successRate = total > 0 ? (float)totalMessages / total * 100.0 : 0.0;

  Serial.println("MESSAGE STATISTICS:");
  Serial.printf("   Total processed: %lu\n", totalMessages);
  Serial.printf("   Invalid/rejected: %lu\n", invalidMessages);
  Serial.printf("   Success rate: %.1f%%\n", successRate);
  Serial.printf("   Depth requests: %lu\n", depthRequests);
  Serial.printf("   Latest depth: %.2f m\n", currentDepth.currentDepth);
  if (currentGPS.valid)
    Serial.printf("   Latest GPS: Lat %.7f, Lon %.7f, Alt %.3f\n", currentGPS.latitude, currentGPS.longitude, currentGPS.altitude);
  else
    Serial.printf("   Latest GPS: not available\n");
  if (currentCOG.valid)
    Serial.printf("   Latest COG/SOG: COG %.1f deg, SOG %.2f m/s\n", currentCOG.cog, currentCOG.sog);
  if (currentMagVar.valid)
    Serial.printf("   Latest Mag Variation: %.1f deg\n", currentMagVar.variation);
  Serial.printf("   Uptime: %.1f minutes\n", millis() / 60000.0);
}

void printHelp() {
  Serial.println("Available commands:");
  Serial.println("  stats            - Show message statistics");
  Serial.println("  test             - Send test depth data");
  Serial.println("  reset            - Reset message buffer");
  Serial.println("  help             - Show this help");
  Serial.println("  debug nmea on    - Enable NMEA 2000 (CAN) debug output");
  Serial.println("  debug nmea off   - Disable NMEA 2000 (CAN) debug output");
  Serial.println("  debug rs485 on   - Enable RS485 debug output");
  Serial.println("  debug rs485 off  - Disable RS485 debug output");
}

String generateMainPageHtml() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32 NMEA2K/RS485 Gateway</title>
    <style>
      body {
        background: #181a1b;
        color: #f1f1f1;
        font-family: Arial, sans-serif;
        margin: 0;
        padding: 0;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: flex-start;
      }
      h2, h3 { color: #90caf9; }
      .info-box {
        background: #23272a;
        padding: 2em 2.5em;
        border-radius: 12px;
        box-shadow: 0 2px 16px #000a;
        margin: 2em 0 1em 0;
        min-width: 320px;
        width: 90vw;
        max-width: 500px;
        display: flex;
        flex-direction: column;
        gap: 0.8em;
      }
      .deep {
        font-size: 1.2em;
        color: #b0f2bc;
      }
      .ota-form {
        margin: 2em 0 1em 0;
      }
      input[type='file'] {
        color: #f1f1f1;
        background: #181a1b;
        border: 1px solid #444;
        border-radius: 6px;
        padding: 0.5em;
      }
      input[type='submit'] {
        background: #1976d2;
        color: #f1f1f1;
        padding: 0.7em 2em;
        border: none;
        border-radius: 6px;
        font-size: 1em;
        cursor: pointer;
        transition: background 0.2s;
      }
      input[type='submit']:hover {
        background: #1565c0;
      }
      .links {
        margin-top: 1em;
        font-size: 1em;
      }
      a {
        color: #90caf9;
        text-decoration: underline;
        margin: 0 0.5em;
      }
      a:hover { color: #1976d2; }
    </style>
  </head>
  <body>
    <h2>ESP32 NMEA2K to RS485 Gateway</h2>
    <div class="info-box">
      <div><b>Firmware Version:</b> )rawliteral";
    html += SW_VERSION_STRING;
    html += " (Build: ";
    html += SW_BUILD_DATE;
    html += ")</div>\n";
    html += "<div><b>Uptime:</b> ";
    unsigned long ms = millis();
    unsigned long sec = ms / 1000;
    unsigned long min = sec / 60;
    unsigned long hr  = min / 60;
    char uptimeStr[32];
    snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu", hr, min % 60, sec % 60);
    html += uptimeStr;
    html += " (hh:mm:ss)</div>\n";
    html += "<div class=\"deep\"><b>Latest Depth:</b> ";
    if (depth_valid && (millis() - lastDepthReceivedMs < nmeaDepthTimeoutMs)) {
        char depthStr[40];
        snprintf(depthStr, sizeof(depthStr), "%.1f ft (%.2f m) <small>(PGN 128267)</small>", depth_tenths / 10.0, currentDepth.currentDepth);
        html += depthStr;
    } else {
        html += "<i>not available</i>";
    }
    html += "</div>\n";
    html += "</div>\n";
    html += R"rawliteral(
    <div class="ota-form">
      <h3>OTA Firmware Update</h3>
      <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='update'>
        <input type='submit' value='Update'>
      </form>
    </div>
    <div class="links">
      <a href="/logs.html">View Logs</a>
    </div>
  </body>
</html>
)rawliteral";
    return html;
}

const char* ota_upload_form = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32 OTA Update</title>
    <style>
      body {
        background: #181a1b;
        color: #f1f1f1;
        font-family: Arial, sans-serif;
        margin: 0;
        padding: 0;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
      }
      h2 { color: #90caf9; }
      form {
        background: #23272a;
        padding: 2em 2.5em;
        border-radius: 12px;
        box-shadow: 0 2px 16px #000a;
        display: flex;
        flex-direction: column;
        gap: 1em;
        align-items: center;
      }
      input[type='file'] {
        color: #f1f1f1;
        background: #181a1b;
        border: 1px solid #444;
        border-radius: 6px;
        padding: 0.5em;
      }
      input[type='submit'] {
        background: #1976d2;
        color: #f1f1f1;
        padding: 0.7em 2em;
        border: none;
        border-radius: 6px;
        font-size: 1em;
        cursor: pointer;
        transition: background 0.2s;
      }
      input[type='submit']:hover {
        background: #1565c0;
      }
    </style>
  </head>
  <body>
    <h2>ESP32 OTA Firmware Update</h2>
    <form method='POST' action='/update' enctype='multipart/form-data'>
      <input type='file' name='update'>
      <input type='submit' value='Update'>
    </form>
  </body>
</html>
)rawliteral";

const char* custom_404_html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>404 Not Found</title>
    <style>
      body {
        background: #181a1b;
        color: #f1f1f1;
        font-family: Arial, sans-serif;
        margin: 0;
        padding: 0;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
      }
      .notfound-container {
        background: #23272a;
        padding: 2em 2.5em;
        border-radius: 12px;
        box-shadow: 0 2px 16px #000a;
        display: flex;
        flex-direction: column;
        gap: 1em;
        align-items: center;
      }
      h1 {
        color: #ef5350;
        font-size: 2.5em;
        margin: 0 0 0.5em 0;
      }
      p {
        color: #b0bfc7;
        font-size: 1.2em;
        margin: 0.2em 0 0 0;
      }
      a {
        color: #90caf9;
        text-decoration: underline;
        margin-top: 1em;
      }
      a:hover {
        color: #1976d2;
      }
    </style>
  </head>
  <body>
    <div class="notfound-container">
      <h1>404</h1>
      <p>Oops! The page you requested was not found.</p>
      <a href="/">Home</a>
    </div>
  </body>
</html>
)rawliteral";

const char* logs_viewer_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Logs Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      background: #181a1b;
      color: #f1f1f1;
      font-family: monospace, Arial, sans-serif;
      margin: 0;
      padding: 0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: flex-start;
    }
    h2 { color: #90caf9; margin-top: 2rem; }
    #logbox {
      background: #23272a;
      width: 90vw;
      max-width: 900px;
      min-height: 60vh;
      margin-top: 1rem;
      border-radius: 10px;
      box-shadow: 0 2px 16px #000a;
      padding: 1em;
      overflow-x: auto;
      overflow-y: auto;
      font-size: 1em;
      white-space: pre-wrap;
      word-break: break-all;
    }
    .controls {
      display: flex;
      gap: 0.5em;
      margin: 1em 0;
      flex-wrap: wrap;
      justify-content: center;
    }
    button {
      background: #1976d2;
      color: #f1f1f1;
      border: none;
      border-radius: 6px;
      padding: 0.5em 1.5em;
      font-size: 1em;
      cursor: pointer;
      transition: background 0.2s;
    }
    button:hover {
      background: #1565c0;
    }
  </style>
</head>
<body>
  <h2>ESP32 Logs Viewer</h2>
  <div class="controls">
    <button onclick="clearLog()">Clear Display</button>
    <button onclick="toggleDebug('nmea', true)">Enable NMEA2000 Debug</button>
    <button onclick="toggleDebug('nmea', false)">Disable NMEA2000 Debug</button>
    <button onclick="toggleDebug('rs485', true)">Enable RS485 Debug</button>
    <button onclick="toggleDebug('rs485', false)">Disable RS485 Debug</button>
  </div>
  <div id="logbox">Loading logs...</div>
  <script>
    const logbox = document.getElementById('logbox');
    let lastLog = '';

    function fetchLogs() {
      fetch('/logs')
        .then(response => response.text())
        .then(data => {
          if (data !== lastLog) {
            logbox.textContent = data;
            logbox.scrollTop = logbox.scrollHeight;
            lastLog = data;
          }
        })
        .catch(err => {
          logbox.textContent = 'Error loading logs: ' + err;
        });
    }

    function clearLog() {
      logbox.textContent = '';
      lastLog = '';
    }

    // Call endpoint to enable/disable debug flags
    function toggleDebug(type, enable) {
      let url = `/debug?type=${type}&enable=${enable ? '1' : '0'}`;
      fetch(url)
        .then(response => response.text())
        .then(msg => {
          // Optionally, append result to logs
          logbox.textContent += `\n${msg}\n`;
          logbox.scrollTop = logbox.scrollHeight;
        })
        .catch(err => {
          logbox.textContent += `\nError setting debug: ${err}\n`;
        });
    }

    // Poll logs every 2 seconds
    setInterval(fetchLogs, 2000);
    fetchLogs();
  </script>
</body>
</html>
)rawliteral";

// --- Setup ---
void setup() {
  Serial.begin(115200);

  Serial.println("\n\n--- esp32 NMEA2k RS485 gateway ---");
  Serial.printf("FW Version: %s (Build: %s)", SW_VERSION_STRING, SW_BUILD_DATE);
  Serial.println();
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.println("Starting RS485 transceiver...");
  Serial.printf("RS485 initialized at %d baud \n", RS485_BAUDRATE);
  Serial.printf("   RX Pin: GPIO%d, TX Pin: GPIO%d, DE/RE Pin: GPIO%d\n", 
                RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("Starting NMEA 2000...");
  Serial.printf("NMEA 2000 initialized at %d kbps \n", CAN_SPEED);
  Serial.printf("   CAN_TX: GPIO%d, CAN_RX: GPIO%d\n", 
                CAN_TX, CAN_RX);
  if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED))) {
    Serial.println("CAN bus failed to start!");
    while(1);
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Web Server routes
  server.on("/", HTTP_GET, []() {
  server.send(200, "text/html", generateMainPageHtml());
  });

  server.onNotFound([]() {
  server.send(404, "text/html", custom_404_html);
  });
    // OTA handler with styled update success page (matching dark mode OTA form)
  server.on("/update", HTTP_POST, []() {
    const char* update_success_html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>Update Success</title>
    <meta http-equiv="refresh" content="5; url=/" />
    <style>
      body {
        background: #181a1b;
        color: #f1f1f1;
        font-family: Arial, sans-serif;
        margin: 0;
        padding: 0;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
      }
      .success-container {
        background: #23272a;
        padding: 2em 2.5em;
        border-radius: 12px;
        box-shadow: 0 2px 16px #000a;
        display: flex;
        flex-direction: column;
        gap: 1em;
        align-items: center;
      }
      h2 {
        color: #90caf9;
        margin-bottom: 0.5em;
      }
      p {
        color: #b0f2bc;
        margin-top: 0.2em;
      }
    </style>
  </head>
  <body>
    <div class="success-container">
      <h2>Update Success!</h2>
      <p>Rebooting...</p>
    </div>
  </body>
</html>
)rawliteral";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html", update_success_html);
  delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_WRITE){
      /* flashing firmware to ESP*/
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_END){
      if(Update.end(true)){ //true to set the size to the current progress
        Serial.printf("Update Success: %u bytes\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
  });

  server.on("/logs", HTTP_GET, []() {
    String logs;
    for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
        logs += logBuffer[(logHead + i) % LOG_BUFFER_SIZE];
    }
    server.send(200, "text/plain", logs);
  });
  server.on("/logs.html", HTTP_GET, []() {
  server.send(200, "text/html", logs_viewer_html);
  });

  server.on("/debug", HTTP_GET, []() {
  String type = server.hasArg("type") ? server.arg("type") : "";
  String enable = server.hasArg("enable") ? server.arg("enable") : "";
  String msg;
  if (type == "nmea") {
    debugNMEA2000 = (enable == "1");
    msg = "NMEA2000 debug " + String(debugNMEA2000 ? "enabled" : "disabled");
  } else if (type == "rs485") {
    debugRS485 = (enable == "1");
    msg = "RS485 debug " + String(debugRS485 ? "enabled" : "disabled");
  } else {
    msg = "Unknown debug type";
  }
  server.send(200, "text/plain", msg);
  });

  server.on("/rs485logs", HTTP_GET, []() {
    server.send(200, "text/plain", getRS485Logs());
  });

  server.begin();
  Serial.println("HTTP server started");
  Serial.println("System startup complete.");
}

// --- Main Loop ---
void loop() {
  ArduinoOTA.handle();
  canReceiver();
  readAndProcessRS485Message();
  //handleSerialCommands();
  server.handleClient();
}