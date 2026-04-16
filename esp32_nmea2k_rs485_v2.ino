/*
 * esp32 NMEA 2000 → RS485 Gateway
 *
 * Bridges NMEA 2000 water depth (PGN 128267) to the Medallion MMDC
 * proprietary RS485 protocol.
 *
 * Architecture
 * ────────────
 *  Core 1 (HIGH priority) — rs485Task
 *    Owns the MAX485 transceiver exclusively. Blocks on UART byte arrival,
 *    assembles the 4-byte depth request, and fires the 13-byte response
 *    with <1 ms latency. Never touched by the web server or OTA handler.
 *
 *  Core 0 (normal priority) — Arduino loop()
 *    Drains the full TWAI RX queue each iteration, runs the web server,
 *    OTA handler, and serial debug console. Latency spikes here cannot
 *    affect RS485 response timing.
 *
 *  Shared state
 *    depthMutex guards depthTenths / depthValid / lastDepthMs so both
 *    cores can safely read and write without tearing.
 *
 * Hardware
 * ────────
 *  CAN  : TJA1050 transceiver — TX→GPIO5, RX→GPIO4, 250 kbps
 *  RS485: MAX485 transceiver  — TX→GPIO17, RX→GPIO16, DE/RE→GPIO21, 76800 baud
 *  WiFi : Soft-AP  SSID "nmea2k_rs485_gw" / pw "123456789"
 *  OTA  : ArduinoOTA + HTTP POST /update
 *  Logs : HTTP GET /logs.html  (live-polling log viewer)
 *
 * Author : caseyn
 * Version: 2.0.0  (2026-04-16)
 */

#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>

// ── Version ──────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.0.0"
#define SW_BUILD_DATE      "2026-04-16"

// ── CAN (NMEA 2000) ──────────────────────────────────────────────────────────
#define CAN_TX_PIN   5
#define CAN_RX_PIN   4
#define CAN_SPEED_KBPS 250

// ── RS485 (MAX485 / HW-097) ──────────────────────────────────────────────────
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 21
#define RS485_BAUDRATE  76800

// ── RS485 protocol constants ─────────────────────────────────────────────────
#define REQUEST_LEN       4
#define RESPONSE_LEN      13
#define MSG_TYPE_REQUEST  0x04
#define MSG_CMD_DEPTH     0x09

// ── Depth staleness timeout ───────────────────────────────────────────────────
#define DEPTH_STALE_MS    5000   // treat depth as invalid after 5 s without CAN update

// ── WiFi AP credentials ───────────────────────────────────────────────────────
static const char *AP_SSID = "nmea2k_rs485_gw";
static const char *AP_PASS = "123456789";

// ── Log ring buffer ───────────────────────────────────────────────────────────
#define LOG_BUF_SIZE 4096
static char     logBuf[LOG_BUF_SIZE];
static size_t   logHead = 0;
static SemaphoreHandle_t logMutex;   // guards logBuf / logHead

// ── Shared depth state ────────────────────────────────────────────────────────
static SemaphoreHandle_t depthMutex;
static volatile uint16_t depthTenths    = 0;   // tenths of a foot
static volatile bool     depthValid     = false;
static volatile uint32_t lastDepthMs    = 0;

// ── RS485 toggle bit (per-response counter seen by MMDC) ─────────────────────
static volatile uint8_t  toggleBit      = 0;

// ── Debug flags (volatile: written from Core 0, read from both) ──────────────
static volatile bool dbgNMEA  = false;
static volatile bool dbgRS485 = false;

// ── Stats (Core 0 only, no mutex needed) ────────────────────────────────────
static uint32_t statDepthRx   = 0;   // PGN 128267 frames received
static uint32_t statRS485Req  = 0;   // depth requests answered
static uint32_t statRS485Bad  = 0;   // requests with bad checksum

HardwareSerial RS485Serial(2);
WebServer      server(80);


// ═══════════════════════════════════════════════════════════════════════════════
// Logging (thread-safe)
// ═══════════════════════════════════════════════════════════════════════════════

static void logMsg(const char *fmt, ...) {
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (len <= 0) return;

    Serial.write((uint8_t *)tmp, len);

    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int i = 0; i < len; i++) {
            logBuf[logHead] = tmp[i];
            logHead = (logHead + 1) % LOG_BUF_SIZE;
        }
        xSemaphoreGive(logMutex);
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// Checksum (8-bit two's complement, same formula used by MMDC)
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t calcChecksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return (~sum + 1) & 0xFF;
}

static bool verifyChecksum(const uint8_t *msg, uint8_t len) {
    if (len < 2) return false;
    return calcChecksum(msg, len - 1) == msg[len - 1];
}


// ═══════════════════════════════════════════════════════════════════════════════
// RS485 transmit helpers (inline, called only from rs485Task on Core 1)
// ═══════════════════════════════════════════════════════════════════════════════

static inline void rs485Tx() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(RS485_DE_RE_PIN, LOW);  }

static void sendDepthResponse() {
    // Snapshot shared depth state under mutex
    uint16_t d = 0;
    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
        d = stale ? 0 : depthTenths;
        xSemaphoreGive(depthMutex);
    }

    uint8_t resp[RESPONSE_LEN] = {
        RESPONSE_LEN,        // [0]  length
        MSG_CMD_DEPTH,       // [1]  command 0x09
        0x14,                // [2]  sub-command
        0xAA,                // [3]  status/ID
        (uint8_t)(d & 0xFF), // [4]  depth low byte
        (uint8_t)(d >> 8),   // [5]  depth high byte
        0xFF, 0x03,          // [6-7]  filler
        0xFF, 0x03,          // [8-9]  filler
        toggleBit,           // [10] toggle
        0x02,                // [11] filler
        0x00                 // [12] checksum placeholder
    };
    resp[12] = calcChecksum(resp, 12);
    toggleBit ^= 1;

    rs485Tx();
    delayMicroseconds(200);           // line-turnaround settle
    RS485Serial.write(resp, RESPONSE_LEN);
    RS485Serial.flush();              // wait for last byte to leave shift register
    delayMicroseconds(100);           // extra guard before releasing bus
    rs485Rx();

    statRS485Req++;

    if (dbgRS485) {
        char hex[RESPONSE_LEN * 3 + 32];
        int n = snprintf(hex, sizeof(hex), "[RS485] TX(%u): ", statRS485Req);
        for (int i = 0; i < RESPONSE_LEN; i++)
            n += snprintf(hex + n, sizeof(hex) - n, "%02X ", resp[i]);
        hex[n++] = '\n'; hex[n] = '\0';
        logMsg("%s", hex);
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// RS485 task — Core 1, priority 10
// Owns all RS485 I/O. Never yields for >a few hundred µs.
// ═══════════════════════════════════════════════════════════════════════════════

static void rs485Task(void *) {
    uint8_t  buf[REQUEST_LEN];
    uint8_t  idx      = 0;
    uint8_t  expected = 0;

    for (;;) {
        // Block until a byte arrives (10 ms tick ceiling keeps watchdog happy)
        if (!RS485Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint8_t b = RS485Serial.read();

        if (idx == 0) {
            // First byte is the length field
            if (b < 4 || b > REQUEST_LEN) {
                // Not a valid start — discard and stay ready
                continue;
            }
            expected = b;
        }

        buf[idx++] = b;

        if (idx < expected) continue;   // message not complete yet

        // Full message received — validate and respond
        if (verifyChecksum(buf, expected)) {
            if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_DEPTH) {
                if (dbgRS485) {
                    logMsg("[RS485] RX depth req: %02X %02X %02X %02X\n",
                           buf[0], buf[1], buf[2], buf[3]);
                }
                sendDepthResponse();
            } else {
                statRS485Bad++;
                if (dbgRS485) logMsg("[RS485] Unknown cmd: %02X %02X\n", buf[0], buf[1]);
            }
        } else {
            statRS485Bad++;
            if (dbgRS485) {
                logMsg("[RS485] Checksum fail: %02X %02X %02X %02X\n",
                       buf[0], buf[1], buf[2], buf[3]);
            }
        }

        idx = expected = 0;  // reset for next message
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// NMEA 2000 / CAN PGN handlers (called from Core 0 loop)
// ═══════════════════════════════════════════════════════════════════════════════

static void handleDepth(const CanFrame &f) {
    if (f.data_length_code < 7) return;

    uint32_t rawM   = (uint32_t)f.data[1] | ((uint32_t)f.data[2] << 8) | ((uint32_t)f.data[3] << 16);
    int16_t  rawOff = (int16_t)((uint16_t)f.data[4] | ((uint16_t)f.data[5] << 8));
    float    depthM = rawM * 0.01f;
    float    depthFt= depthM * 3.28084f;
    uint16_t tenths = (uint16_t)(depthFt * 10.0f + 0.5f);

    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        depthTenths  = tenths;
        depthValid   = true;
        lastDepthMs  = millis();
        xSemaphoreGive(depthMutex);
    }
    statDepthRx++;

    if (dbgNMEA) {
        logMsg("PGN 128267 depth: %.2f m / %.1f ft (tenths=%u)\n",
               depthM, depthFt, tenths);
    }
}

// Lightweight stubs for other PGNs — just log when debug on, no stored state needed
static void handlePos129025(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 8) return;
    int32_t lon = (int32_t)((uint32_t)f.data[0] | ((uint32_t)f.data[1]<<8) | ((uint32_t)f.data[2]<<16) | ((uint32_t)f.data[3]<<24));
    int32_t lat = (int32_t)((uint32_t)f.data[4] | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24));
    logMsg("PGN 129025 pos: lat=%.7f lon=%.7f\n", lat*1e-7, lon*1e-7);
}

static void handleCOG(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 6) return;
    float cog = ((uint16_t)f.data[2] | ((uint16_t)f.data[3]<<8)) * 0.0001f * 57.2957795f;
    float sog = ((uint16_t)f.data[4] | ((uint16_t)f.data[5]<<8)) * 0.01f;
    logMsg("PGN 129026 COG=%.1f° SOG=%.2f m/s\n", cog, sog);
}

static void handleMagVar(const CanFrame &f) {
    if (!dbgNMEA || f.data_length_code < 6) return;
    float var = (int16_t)((uint16_t)f.data[4] | ((uint16_t)f.data[5]<<8)) * 0.0001f * 57.2957795f;
    logMsg("PGN 127258 MagVar=%.1f°\n", var);
}

// ── PGN dispatch table ────────────────────────────────────────────────────────
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
    return (pf < 240) ? ((uint32_t)dp << 16) | ((uint32_t)pf << 8)
                      : ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps;
}

// Drain the full TWAI RX queue — call every loop() iteration
static void drainCAN() {
    CanFrame f;
    while (ESP32Can.readFrame(f)) {
        uint32_t pgn = extractPGN(f.identifier);
        for (int i = 0; i < PGN_TABLE_SIZE; i++) {
            if (pgnTable[i].pgn == pgn) {
                pgnTable[i].handler(f);
                break;
            }
        }
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// Serial command handler (Core 0)
// ═══════════════════════════════════════════════════════════════════════════════

static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if      (cmd == "debug nmea on")   { dbgNMEA  = true;  logMsg("NMEA debug ON\n"); }
    else if (cmd == "debug nmea off")  { dbgNMEA  = false; logMsg("NMEA debug OFF\n"); }
    else if (cmd == "debug rs485 on")  { dbgRS485 = true;  logMsg("RS485 debug ON\n"); }
    else if (cmd == "debug rs485 off") { dbgRS485 = false; logMsg("RS485 debug OFF\n"); }
    else if (cmd == "depth") {
        if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
            logMsg("depth: %.1f ft (%s)\n", depthTenths / 10.0f, stale ? "STALE" : "fresh");
            xSemaphoreGive(depthMutex);
        }
    }
    else if (cmd == "stats") {
        logMsg("stats: depthRx=%lu  rs485Req=%lu  rs485Bad=%lu  uptime=%.1f min\n",
               statDepthRx, statRS485Req, statRS485Bad, millis() / 60000.0f);
    }
    else if (cmd == "help") {
        logMsg("commands: depth | stats | debug nmea on/off | debug rs485 on/off | help\n");
    }
    else {
        logMsg("unknown command (try 'help')\n");
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// Web server HTML pages
// ═══════════════════════════════════════════════════════════════════════════════

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


// ── /status JSON ──────────────────────────────────────────────────────────────
static void handleStatusJSON() {
    uint16_t d = 0; bool stale = true;
    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
        d = depthTenths;
        xSemaphoreGive(depthMutex);
    }
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
        d / 10.0f,
        stale ? "false" : "true",
        statDepthRx, statRS485Req, statRS485Bad,
        dbgNMEA  ? "true" : "false",
        dbgRS485 ? "true" : "false"
    );
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", buf);
}


// ═══════════════════════════════════════════════════════════════════════════════
// setup()
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(100);

    logMutex   = xSemaphoreCreateMutex();
    depthMutex = xSemaphoreCreateMutex();

    logMsg("\n\n=== esp32 NMEA2k→RS485 Gateway %s (%s) ===\n",
           SW_VERSION_STRING, SW_BUILD_DATE);

    // ── RS485 ────────────────────────────────────────────────────────────────
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    rs485Rx();
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    logMsg("RS485 ready  %d baud  RX=GPIO%d TX=GPIO%d DE/RE=GPIO%d\n",
           RS485_BAUDRATE, RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);

    // ── CAN / NMEA 2000 ───────────────────────────────────────────────────────
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        logMsg("FATAL: CAN bus failed to start\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    logMsg("NMEA2000 ready  %d kbps  TX=GPIO%d RX=GPIO%d\n",
           CAN_SPEED_KBPS, CAN_TX_PIN, CAN_RX_PIN);

    // ── WiFi AP ───────────────────────────────────────────────────────────────
    WiFi.softAP(AP_SSID, AP_PASS);
    logMsg("WiFi AP  SSID=%s  IP=%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    // ── OTA ───────────────────────────────────────────────────────────────────
    ArduinoOTA.setHostname("nmea2k-rs485-gw");
    ArduinoOTA.begin();

    // ── Web routes ────────────────────────────────────────────────────────────
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", OTA_HTML);
    });

    server.on("/update", HTTP_POST,
        []() {
            server.sendHeader("Connection", "close");
            server.send_P(200, "text/html", OTA_SUCCESS_HTML);
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP.restart();
        },
        []() {
            HTTPUpload &up = server.upload();
            if      (up.status == UPLOAD_FILE_START)  { Update.begin(UPDATE_SIZE_UNKNOWN); }
            else if (up.status == UPLOAD_FILE_WRITE)  { Update.write(up.buf, up.currentSize); }
            else if (up.status == UPLOAD_FILE_END)    { Update.end(true); }
        }
    );

    server.on("/logs", HTTP_GET, []() {
        String out;
        out.reserve(LOG_BUF_SIZE);
        if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            for (size_t i = 0; i < LOG_BUF_SIZE; i++)
                out += logBuf[(logHead + i) % LOG_BUF_SIZE];
            xSemaphoreGive(logMutex);
        }
        server.send(200, "text/plain", out);
    });

    server.on("/logs.html", HTTP_GET, []() {
        server.send_P(200, "text/html", LOGS_HTML);
    });

    server.on("/status", HTTP_GET, handleStatusJSON);

    server.on("/debug", HTTP_GET, []() {
        String type   = server.hasArg("type")   ? server.arg("type")   : "";
        String enable = server.hasArg("enable")  ? server.arg("enable") : "";
        String msg;
        if (type == "nmea") {
            dbgNMEA = (enable == "1");
            msg = String("NMEA debug ") + (dbgNMEA ? "ON" : "OFF");
        } else if (type == "rs485") {
            dbgRS485 = (enable == "1");
            msg = String("RS485 debug ") + (dbgRS485 ? "ON" : "OFF");
        } else {
            msg = "unknown type";
        }
        server.send(200, "text/plain", msg);
    });

    server.onNotFound([]() {
        server.send_P(404, "text/html", NOT_FOUND_HTML);
    });

    server.begin();
    logMsg("HTTP server started\n");

    // ── RS485 real-time task on Core 1 ────────────────────────────────────────
    // Stack 2 kB is plenty; priority 10 beats any Arduino default task.
    xTaskCreatePinnedToCore(
        rs485Task,       // function
        "rs485",         // name
        2048,            // stack bytes
        nullptr,         // parameter
        10,              // priority (0=lowest, configMAX_PRIORITIES-1=highest)
        nullptr,         // handle (not needed)
        1                // Core 1
    );

    logMsg("rs485Task pinned to Core 1\n");
    logMsg("Setup complete.\n");
}


// ═══════════════════════════════════════════════════════════════════════════════
// loop() — Core 0 only
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
    ArduinoOTA.handle();
    drainCAN();           // drain full TWAI queue, not just one frame
    handleSerial();
    server.handleClient();
}
