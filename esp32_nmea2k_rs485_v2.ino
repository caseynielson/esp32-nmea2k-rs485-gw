/*
 * esp32 NMEA 2000 → RS485 Gateway
 *
 * Bridges NMEA 2000 water depth (PGN 128267) to the Medallion MMDC
 * proprietary RS485 protocol.
 *
 * Architecture
 * ────────────
 *  No extra FreeRTOS tasks or core pinning — all work happens in loop().
 *
 *  RS485 responsiveness:
 *    A HardwareSerial UART interrupt buffers incoming bytes automatically.
 *    loop() checks RS485Serial.available() first, before anything else,
 *    and responds immediately when a complete request is assembled.
 *    The web server and OTA handler are time-sliced with a 5 ms budget
 *    each via non-blocking millis() guards so they cannot stall the loop.
 *
 *  CAN / NMEA 2000:
 *    drainCAN() empties the full TWAI RX queue every iteration.
 *
 *  Serial output:
 *    All Serial I/O stays on the Arduino loop task (single owner).
 *    logMsg() writes to a ring buffer; drainLogToSerial() flushes it
 *    to Serial at the top of each loop() iteration.
 *
 * Hardware
 * ────────
 *  CAN  : TJA1050 transceiver — TX→GPIO5, RX→GPIO4, 250 kbps
 *  RS485: MAX485 transceiver  — TX→GPIO17, RX→GPIO16, DE/RE→GPIO21, 76800 baud
 *  WiFi : Soft-AP  SSID "nmea2k_rs485_gw" / pw "123456789"
 *  OTA  : HTTP POST /update (browser-based, no ArduinoOTA/mDNS)
 *  Logs : HTTP GET /logs.html  (live-polling log viewer)
 *  JSON : HTTP GET /status
 *
 * Author : caseyn
 * Version: 2.1.0  (2026-04-16)
 */

#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.1.5"
#define SW_BUILD_DATE      "2026-04-16"

// ── CAN (NMEA 2000) ───────────────────────────────────────────────────────────
#define CAN_TX_PIN      5
#define CAN_RX_PIN      4
#define CAN_SPEED_KBPS  250

// ── RS485 (MAX485) ────────────────────────────────────────────────────────────
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 21
#define RS485_BAUDRATE  76800

// ── RS485 protocol ────────────────────────────────────────────────────────────
#define REQUEST_LEN       4
#define RESPONSE_LEN      13
#define MSG_TYPE_REQUEST  0x04
#define MSG_CMD_DEPTH     0x09

// ── Depth staleness ───────────────────────────────────────────────────────────
#define DEPTH_STALE_MS  5000

// ── WiFi AP ───────────────────────────────────────────────────────────────────
static const char *AP_SSID = "nmea2k_rs485_gw";
static const char *AP_PASS = "123456789";

// ── Log ring buffer (single-owner: loop task only) ────────────────────────────
#define LOG_BUF_SIZE 4096
static char   logBuf[LOG_BUF_SIZE];
static size_t logHead = 0;   // write pointer
static size_t logTail = 0;   // read  pointer (Serial drain)

// ── Depth state ───────────────────────────────────────────────────────────────
static uint16_t depthTenths   = 0;
static bool     depthValid    = false;
static uint32_t lastDepthMs   = 0;

// ── RS485 toggle bit ─────────────────────────────────────────────────────────
static uint8_t toggleBit = 0;

// ── Debug flags ───────────────────────────────────────────────────────────────
static bool dbgNMEA  = false;
static bool dbgRS485 = false;

// ── Stats ─────────────────────────────────────────────────────────────────────
static uint32_t statDepthRx  = 0;
static uint32_t statRS485Req = 0;
static uint32_t statRS485Bad = 0;

HardwareSerial RS485Serial(2);
WebServer      server(80);


// ═════════════════════════════════════════════════════════════════════════════
// Logging — ring buffer only, no direct Serial calls outside loop()
// ═════════════════════════════════════════════════════════════════════════════

static void logMsg(const char *fmt, ...) {
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    for (int i = 0; i < len; i++) {
        logBuf[logHead] = tmp[i];
        logHead = (logHead + 1) % LOG_BUF_SIZE;
        // If we lap the tail, advance it (drop oldest byte)
        if (logHead == logTail)
            logTail = (logTail + 1) % LOG_BUF_SIZE;
    }
}

// Called first in loop() — drains new bytes to Serial in one shot
static void drainLogToSerial() {
    while (logTail != logHead) {
        Serial.write((uint8_t)logBuf[logTail]);
        logTail = (logTail + 1) % LOG_BUF_SIZE;
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// Checksum
// ═════════════════════════════════════════════════════════════════════════════

static uint8_t calcChecksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return (~sum + 1) & 0xFF;
}

static bool verifyChecksum(const uint8_t *msg, uint8_t len) {
    if (len < 2) return false;
    return calcChecksum(msg, len - 1) == msg[len - 1];
}


// ═════════════════════════════════════════════════════════════════════════════
// RS485 — request parser + response sender (called from loop())
// ═════════════════════════════════════════════════════════════════════════════

static inline void rs485Tx() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(RS485_DE_RE_PIN, LOW);  }

static void sendDepthResponse() {
    bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
    uint16_t d = stale ? 0 : depthTenths;

    uint8_t resp[RESPONSE_LEN] = {
        RESPONSE_LEN,
        MSG_CMD_DEPTH,
        0x14,
        0xAA,
        (uint8_t)(d & 0xFF),
        (uint8_t)(d >> 8),
        0xFF, 0x03,
        0xFF, 0x03,
        toggleBit,
        0x02,
        0x00
    };
    resp[12] = calcChecksum(resp, 12);
    toggleBit ^= 1;
    statRS485Req++;

    rs485Tx();
    delayMicroseconds(200);
    RS485Serial.write(resp, RESPONSE_LEN);
    RS485Serial.flush();
    delayMicroseconds(100);
    rs485Rx();

    if (dbgRS485) {
        logMsg("[RS485] TX #%lu depth=%.1fft%s: ", statRS485Req, d / 10.0f,
               stale ? "(stale)" : "");
        for (int i = 0; i < RESPONSE_LEN; i++) logMsg("%02X ", resp[i]);
        logMsg("\n");
    }
}

// Assemble incoming RS485 bytes and respond when a complete request arrives.
// Called at the top of loop() before anything else — UART HW buffer holds
// bytes safely between calls, so no bytes are lost even if loop() is slow.
static void handleRS485() {
    static uint8_t buf[REQUEST_LEN];
    static uint8_t idx      = 0;
    static uint8_t expected = 0;

    while (RS485Serial.available()) {
        uint8_t b = RS485Serial.read();

        if (idx == 0) {
            // Length byte — validate before accepting
            if (b < 4 || b > REQUEST_LEN) continue;
            expected = b;
        }
        buf[idx++] = b;

        if (idx < expected) continue;

        // Full message — validate and respond
        if (verifyChecksum(buf, expected)) {
            if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_DEPTH) {
                if (dbgRS485) logMsg("[RS485] RX req: %02X %02X %02X %02X\n",
                                     buf[0], buf[1], buf[2], buf[3]);
                sendDepthResponse();
            } else {
                statRS485Bad++;
                if (dbgRS485) logMsg("[RS485] unknown cmd %02X %02X\n", buf[0], buf[1]);
            }
        } else {
            statRS485Bad++;
            if (dbgRS485) logMsg("[RS485] checksum fail: %02X %02X %02X %02X\n",
                                 buf[0], buf[1], buf[2], buf[3]);
        }

        idx = expected = 0;
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// NMEA 2000 / CAN PGN handlers
// ═════════════════════════════════════════════════════════════════════════════

static void handleDepth(const CanFrame &f) {
    if (f.data_length_code < 7) return;
    uint32_t rawM  = (uint32_t)f.data[1] | ((uint32_t)f.data[2] << 8) | ((uint32_t)f.data[3] << 16);
    float    depthM  = rawM * 0.01f;
    float    depthFt = depthM * 3.28084f;
    depthTenths  = (uint16_t)(depthFt * 10.0f + 0.5f);
    depthValid   = true;
    lastDepthMs  = millis();
    statDepthRx++;
    if (dbgNMEA) logMsg("PGN 128267 depth: %.2fm / %.1fft\n", depthM, depthFt);
}

static void handlePos129025(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 8) return;
    int32_t lon = (int32_t)((uint32_t)f.data[0]|((uint32_t)f.data[1]<<8)|((uint32_t)f.data[2]<<16)|((uint32_t)f.data[3]<<24));
    int32_t lat = (int32_t)((uint32_t)f.data[4]|((uint32_t)f.data[5]<<8)|((uint32_t)f.data[6]<<16)|((uint32_t)f.data[7]<<24));
    logMsg("PGN 129025 pos: lat=%.7f lon=%.7f\n", lat*1e-7, lon*1e-7);
}

static void handleCOG(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 6) return;
    float cog = ((uint16_t)f.data[2]|((uint16_t)f.data[3]<<8)) * 0.0001f * 57.2957795f;
    float sog = ((uint16_t)f.data[4]|((uint16_t)f.data[5]<<8)) * 0.01f;
    logMsg("PGN 129026 COG=%.1f° SOG=%.2fm/s\n", cog, sog);
}

static void handleMagVar(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 6) return;
    float var = (int16_t)((uint16_t)f.data[4]|((uint16_t)f.data[5]<<8)) * 0.0001f * 57.2957795f;
    logMsg("PGN 127258 MagVar=%.1f°\n", var);
}

typedef void (*PGNHandler)(const CanFrame &);
struct PGNEntry { uint32_t pgn; PGNHandler handler; };
static const PGNEntry pgnTable[] = {
    { 128267, handleDepth     },
    { 129025, handlePos129025 },
    { 129026, handleCOG       },
    { 127258, handleMagVar    },
};
static const int PGN_TABLE_SIZE = sizeof(pgnTable) / sizeof(PGNEntry);

static uint32_t extractPGN(uint32_t id) {
    uint8_t dp = (id >> 24) & 0x01;
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >>  8) & 0xFF;
    return (pf < 240) ? ((uint32_t)dp<<16)|((uint32_t)pf<<8)
                      : ((uint32_t)dp<<16)|((uint32_t)pf<<8)|ps;
}

static void drainCAN() {
    CanFrame f;
    while (ESP32Can.readFrame(f)) {
        uint32_t pgn = extractPGN(f.identifier);
        for (int i = 0; i < PGN_TABLE_SIZE; i++) {
            if (pgnTable[i].pgn == pgn) { pgnTable[i].handler(f); break; }
        }
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// Serial — debug toggle only (non-blocking)
// Full debug control available via /logs.html on the WiFi AP.
// ═════════════════════════════════════════════════════════════════════════════

static void handleSerial() {
    static String cmd;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { cmd += c; continue; }
        cmd.trim();
        if      (cmd == "nmea on")   { dbgNMEA  = true;  logMsg("NMEA debug ON\n"); }
        else if (cmd == "nmea off")  { dbgNMEA  = false; logMsg("NMEA debug OFF\n"); }
        else if (cmd == "rs485 on")  { dbgRS485 = true;  logMsg("RS485 debug ON\n"); }
        else if (cmd == "rs485 off") { dbgRS485 = false; logMsg("RS485 debug OFF\n"); }
        cmd = "";
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// Web server + OTA — called every loop() iteration
// Both are non-blocking when idle. handleRS485() runs first so any
// RS485 request is serviced before these get a turn.
// ═════════════════════════════════════════════════════════════════════════════

static void handleWeb() { server.handleClient(); }


// ═════════════════════════════════════════════════════════════════════════════
// Web server HTML
// ═════════════════════════════════════════════════════════════════════════════

static const char OTA_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>ESP32 OTA Update</title>
<style>
body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;margin:0}
h2{color:#90caf9}
form{background:#23272a;padding:2em 2.5em;border-radius:12px;box-shadow:0 2px 16px #000a;display:flex;flex-direction:column;gap:1em;align-items:center}
input[type=file]{color:#f1f1f1;background:#181a1b;border:1px solid #444;border-radius:6px;padding:.5em}
input[type=submit]{background:#1976d2;color:#f1f1f1;padding:.7em 2em;border:none;border-radius:6px;font-size:1em;cursor:pointer}
input[type=submit]:hover{background:#1565c0}
</style></head>
<body><h2>ESP32 OTA Firmware Update</h2>
<form method='POST' action='/update' enctype='multipart/form-data'>
<input type='file' name='update'><input type='submit' value='Update'>
</form></body></html>
)html";

static const char OTA_SUCCESS_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>Update Success</title><meta http-equiv="refresh" content="5;url=/">
<style>body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
.box{background:#23272a;padding:2em 2.5em;border-radius:12px;text-align:center}
h2{color:#90caf9}p{color:#b0f2bc}</style></head>
<body><div class="box"><h2>Update Success!</h2><p>Rebooting…</p></div></body></html>
)html";

static const char LOGS_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>Gateway Logs</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{background:#181a1b;color:#f1f1f1;font-family:monospace,Arial;margin:0;padding:0;display:flex;flex-direction:column;align-items:center}
h2{color:#90caf9;margin-top:2rem}
#lb{background:#23272a;width:90vw;max-width:900px;min-height:60vh;margin:1rem 0;border-radius:10px;box-shadow:0 2px 16px #000a;padding:1em;overflow:auto;white-space:pre-wrap;word-break:break-all;font-size:.95em}
.ctrl{display:flex;gap:.5em;margin:.5em 0;flex-wrap:wrap;justify-content:center}
button{background:#1976d2;color:#f1f1f1;border:none;border-radius:6px;padding:.5em 1.2em;font-size:.95em;cursor:pointer}
button:hover{background:#1565c0}
</style></head>
<body>
<h2>Gateway Logs</h2>
<div class="ctrl">
  <button onclick="fetch('/debug?type=nmea&enable=1').then(r=>r.text()).then(t=>append(t))">NMEA Debug ON</button>
  <button onclick="fetch('/debug?type=nmea&enable=0').then(r=>r.text()).then(t=>append(t))">NMEA Debug OFF</button>
  <button onclick="fetch('/debug?type=rs485&enable=1').then(r=>r.text()).then(t=>append(t))">RS485 Debug ON</button>
  <button onclick="fetch('/debug?type=rs485&enable=0').then(r=>r.text()).then(t=>append(t))">RS485 Debug OFF</button>
  <button onclick="document.getElementById('lb').textContent=''">Clear Display</button>
</div>
<div id="lb">Connecting…</div>
<script>
const lb=document.getElementById('lb');
let last='';
function append(t){lb.textContent+='\n'+t;lb.scrollTop=lb.scrollHeight;}
function poll(){fetch('/logs').then(r=>r.text()).then(d=>{if(d!==last){lb.textContent=d;lb.scrollTop=lb.scrollHeight;last=d;}}).catch(()=>{})}
setInterval(poll,2000);poll();
</script>
</body></html>
)html";

static const char NOT_FOUND_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>404</title>
<style>body{background:#181a1b;color:#f1f1f1;font-family:Arial;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
.box{background:#23272a;padding:2em 2.5em;border-radius:12px;text-align:center}
h1{color:#ef5350;font-size:2.5em}p{color:#b0bfc7}a{color:#90caf9}</style></head>
<body><div class="box"><h1>404</h1><p>Page not found.</p><a href="/">Home</a></div></body></html>
)html";

static void handleStatusJSON() {
    bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"version\":\"%s\","
        "\"uptime_s\":%lu,"
        "\"depth_ft\":%.1f,"
        "\"depth_valid\":%s,"
        "\"depth_rx\":%lu,"
        "\"rs485_req\":%lu,"
        "\"rs485_bad\":%lu,"
        "\"dbg_nmea\":%s,"
        "\"dbg_rs485\":%s}",
        SW_VERSION_STRING,
        millis() / 1000UL,
        depthTenths / 10.0f,
        stale ? "false" : "true",
        statDepthRx, statRS485Req, statRS485Bad,
        dbgNMEA  ? "true" : "false",
        dbgRS485 ? "true" : "false"
    );
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", buf);
}


// ═════════════════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    delay(500);   // let USB-CDC settle after upload reset
    Serial.begin(115200);
    delay(100);

    // Direct print for startup banner — logMsg() won't drain until loop() starts
    Serial.printf("\n\n=== esp32 NMEA2k\u2192RS485 Gateway %s (%s) ===\n",
                  SW_VERSION_STRING, SW_BUILD_DATE);

    // RS485
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    rs485Rx();
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    logMsg("RS485 ready  %d baud  RX=GPIO%d TX=GPIO%d DE/RE=GPIO%d\n",
           RS485_BAUDRATE, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);

    // CAN / NMEA 2000
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        logMsg("FATAL: CAN bus failed to start\n");
        while (1) delay(1000);
    }
    logMsg("NMEA2000 ready  %d kbps  TX=GPIO%d RX=GPIO%d\n",
           CAN_SPEED_KBPS, CAN_TX_PIN, CAN_RX_PIN);

    // WiFi AP
    WiFi.softAP(AP_SSID, AP_PASS);
    logMsg("WiFi AP  SSID=%s  IP=%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    // Web routes
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", OTA_HTML);
    });
    server.on("/update", HTTP_POST,
        []() {
            server.sendHeader("Connection", "close");
            server.send_P(200, "text/html", OTA_SUCCESS_HTML);
            delay(500);
            ESP.restart();
        },
        []() {
            HTTPUpload &up = server.upload();
            if      (up.status == UPLOAD_FILE_START) { Update.begin(UPDATE_SIZE_UNKNOWN); }
            else if (up.status == UPLOAD_FILE_WRITE) { Update.write(up.buf, up.currentSize); }
            else if (up.status == UPLOAD_FILE_END)   { Update.end(true); }
        }
    );
    server.on("/logs", HTTP_GET, []() {
        // Serve ring buffer contents as plain text
        String out;
        out.reserve(LOG_BUF_SIZE);
        size_t i = logTail;
        while (i != logHead) {
            out += logBuf[i];
            i = (i + 1) % LOG_BUF_SIZE;
        }
        server.send(200, "text/plain", out);
    });
    server.on("/logs.html", HTTP_GET, []() {
        server.send_P(200, "text/html", LOGS_HTML);
    });
    server.on("/status",    HTTP_GET, handleStatusJSON);
    server.on("/debug", HTTP_GET, []() {
        String type   = server.hasArg("type")   ? server.arg("type")   : "";
        String enable = server.hasArg("enable")  ? server.arg("enable") : "";
        String msg;
        if      (type == "nmea")  { dbgNMEA  = (enable=="1"); msg = "NMEA debug "  + String(dbgNMEA  ? "ON":"OFF"); }
        else if (type == "rs485") { dbgRS485 = (enable=="1"); msg = "RS485 debug " + String(dbgRS485 ? "ON":"OFF"); }
        else                      { msg = "unknown type"; }
        server.send(200, "text/plain", msg);
    });
    server.onNotFound([]() {
        server.send_P(404, "text/html", NOT_FOUND_HTML);
    });
    server.begin();

    logMsg("Setup complete. Type 'help' for commands.\n");
}


// ═════════════════════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    handleRS485();       // 1st: check for MMDC request and respond immediately
    drainLogToSerial();  // flush log buffer → Serial (single Serial owner)
    drainCAN();          // drain full TWAI RX queue
    handleSerial();      // serial debug toggle
    handleWeb();         // web server
}
