/*
 * esp32 NMEA 2000 → RS485 Gateway
 *
 * Bridges NMEA 2000 water depth (PGN 128267) to the Medallion MMDC
 * proprietary RS485 protocol.
 *
 * Hardware
 * ────────
 *  CAN  : TJA1050 — TX→GPIO5, RX→GPIO4, 250 kbps
 *  RS485: MAX485  — TX→GPIO17, RX→GPIO16, DE/RE→GPIO21, 76800 baud
 *  WiFi : Soft-AP  SSID "nmea2k_rs485_gw" / pw "123456789"
 *
 * Web interface (connect to WiFi AP, open browser)
 * ────────────────────────────────────────────────
 *  http://192.168.4.1/          — firmware update
 *  http://192.168.4.1/status    — JSON status
 *  http://192.168.4.1/logs.html — live log viewer + debug toggles
 *
 * Author : caseyn
 * Version: 2.2.0  (2026-04-16)
 */

#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.2.0"
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
#define REQUEST_LEN      4
#define RESPONSE_LEN     13
#define MSG_TYPE_REQUEST 0x04
#define MSG_CMD_DEPTH    0x09

// ── Depth staleness ───────────────────────────────────────────────────────────
#define DEPTH_STALE_MS   5000

// ── WiFi AP ───────────────────────────────────────────────────────────────────
static const char *AP_SSID = "nmea2k_rs485_gw";
static const char *AP_PASS = "123456789";

// ── Depth state ───────────────────────────────────────────────────────────────
static uint16_t depthTenths = 0;
static bool     depthValid  = false;
static uint32_t lastDepthMs = 0;

// ── RS485 toggle bit ─────────────────────────────────────────────────────────
static uint8_t toggleBit = 0;

// ── Debug flags (toggled via web UI) ─────────────────────────────────────────
static bool dbgNMEA  = false;
static bool dbgRS485 = false;

// ── Stats ─────────────────────────────────────────────────────────────────────
static uint32_t statDepthRx  = 0;
static uint32_t statRS485Req = 0;
static uint32_t statRS485Bad = 0;

// ── Log ring buffer (web only, no Serial) ────────────────────────────────────
#define LOG_BUF_SIZE 2048
static char   logBuf[LOG_BUF_SIZE];
static size_t logHead = 0;

static void logMsg(const char *fmt, ...) {
    char tmp[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    for (int i = 0; i < len; i++) {
        logBuf[logHead] = tmp[i];
        logHead = (logHead + 1) % LOG_BUF_SIZE;
    }
}

HardwareSerial RS485Serial(2);
WebServer      server(80);


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
// RS485
// ═════════════════════════════════════════════════════════════════════════════

static inline void rs485Tx() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(RS485_DE_RE_PIN, LOW);  }

static void sendDepthResponse() {
    bool     stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
    uint16_t d     = stale ? 0 : depthTenths;

    uint8_t resp[RESPONSE_LEN] = {
        RESPONSE_LEN,
        MSG_CMD_DEPTH,
        0x14, 0xAA,
        (uint8_t)(d & 0xFF), (uint8_t)(d >> 8),
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
        logMsg("[RS485] TX #%lu %.1fft%s\n",
               statRS485Req, d / 10.0f, stale ? " (stale)" : "");
    }
}

static void handleRS485() {
    static uint8_t buf[REQUEST_LEN];
    static uint8_t idx      = 0;
    static uint8_t expected = 0;

    while (RS485Serial.available()) {
        uint8_t b = RS485Serial.read();
        if (idx == 0) {
            if (b < 4 || b > REQUEST_LEN) continue;
            expected = b;
        }
        buf[idx++] = b;
        if (idx < expected) continue;

        if (verifyChecksum(buf, expected)) {
            if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_DEPTH) {
                sendDepthResponse();
            } else {
                statRS485Bad++;
            }
        } else {
            statRS485Bad++;
            if (dbgRS485) logMsg("[RS485] bad checksum\n");
        }
        idx = expected = 0;
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// NMEA 2000 / CAN
// ═════════════════════════════════════════════════════════════════════════════

static void handleDepth(const CanFrame &f) {
    if (f.data_length_code < 7) return;
    uint32_t rawM   = (uint32_t)f.data[1] | ((uint32_t)f.data[2]<<8) | ((uint32_t)f.data[3]<<16);
    float    depthM = rawM * 0.01f;
    float    depthFt= depthM * 3.28084f;
    depthTenths = (uint16_t)(depthFt * 10.0f + 0.5f);
    depthValid  = true;
    lastDepthMs = millis();
    statDepthRx++;
    if (dbgNMEA) logMsg("PGN128267 %.2fm / %.1fft\n", depthM, depthFt);
}

typedef void (*PGNHandler)(const CanFrame &);
struct PGNEntry { uint32_t pgn; PGNHandler handler; };
static const PGNEntry pgnTable[] = {
    { 128267, handleDepth },
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
// Web server HTML
// ═════════════════════════════════════════════════════════════════════════════

static const char UPDATE_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>Gateway Firmware Update</title>
<style>
body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;margin:0}
h2{color:#90caf9}
form{background:#23272a;padding:2em 2.5em;border-radius:12px;box-shadow:0 2px 16px #000a;display:flex;flex-direction:column;gap:1em;align-items:center}
input[type=file]{color:#f1f1f1;background:#181a1b;border:1px solid #444;border-radius:6px;padding:.5em}
input[type=submit]{background:#1976d2;color:#f1f1f1;padding:.7em 2em;border:none;border-radius:6px;font-size:1em;cursor:pointer}
input[type=submit]:hover{background:#1565c0}
a{color:#90caf9;margin-top:1em;display:block;text-align:center}
</style></head>
<body><h2>Firmware Update</h2>
<form method='POST' action='/update' enctype='multipart/form-data'>
<input type='file' name='update'><input type='submit' value='Upload &amp; Flash'>
</form>
<a href='/logs.html'>Logs &amp; Debug</a>
<a href='/status'>Status JSON</a>
</body></html>
)html";

static const char UPDATE_OK_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>Update OK</title><meta http-equiv="refresh" content="5;url=/">
<style>body{background:#181a1b;color:#f1f1f1;font-family:Arial;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
.box{background:#23272a;padding:2em 2.5em;border-radius:12px;text-align:center}
h2{color:#90caf9}p{color:#b0f2bc}</style></head>
<body><div class="box"><h2>Update OK</h2><p>Rebooting…</p></div></body></html>
)html";

static const char LOGS_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head><title>Gateway Logs</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{background:#181a1b;color:#f1f1f1;font-family:monospace,Arial;margin:0;padding:0;display:flex;flex-direction:column;align-items:center}
h2{color:#90caf9;margin-top:2rem}
#lb{background:#23272a;width:90vw;max-width:900px;min-height:60vh;margin:1rem 0;border-radius:10px;padding:1em;overflow:auto;white-space:pre-wrap;word-break:break-all;font-size:.95em}
.ctrl{display:flex;gap:.5em;margin:.5em 0;flex-wrap:wrap;justify-content:center}
button{background:#1976d2;color:#f1f1f1;border:none;border-radius:6px;padding:.5em 1.2em;font-size:.95em;cursor:pointer}
button:hover{background:#1565c0}
</style></head>
<body><h2>Gateway Logs</h2>
<div class="ctrl">
  <button onclick="dbg('nmea',1)">NMEA ON</button>
  <button onclick="dbg('nmea',0)">NMEA OFF</button>
  <button onclick="dbg('rs485',1)">RS485 ON</button>
  <button onclick="dbg('rs485',0)">RS485 OFF</button>
  <button onclick="document.getElementById('lb').textContent=''">Clear</button>
  <a href="/" style="color:#90caf9;padding:.5em 1.2em">Update FW</a>
</div>
<div id="lb">Loading…</div>
<script>
const lb=document.getElementById('lb');
let last='';
function dbg(t,e){fetch('/debug?type='+t+'&enable='+e).then(r=>r.text()).then(t=>{lb.textContent+='\n'+t;lb.scrollTop=lb.scrollHeight;})}
function poll(){fetch('/logs').then(r=>r.text()).then(d=>{if(d!==last){lb.textContent=d;lb.scrollTop=lb.scrollHeight;last=d;}}).catch(()=>{})}
setInterval(poll,2000);poll();
</script>
</body></html>
)html";


// ═════════════════════════════════════════════════════════════════════════════
// Web handlers
// ═════════════════════════════════════════════════════════════════════════════

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
        "\"rs485_bad\":%lu}",
        SW_VERSION_STRING,
        millis() / 1000UL,
        depthTenths / 10.0f,
        stale ? "false" : "true",
        statDepthRx, statRS485Req, statRS485Bad
    );
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", buf);
}


// ═════════════════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    // RS485
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    rs485Rx();
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    // WiFi AP
    WiFi.softAP(AP_SSID, AP_PASS);

    // Web routes
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", UPDATE_HTML);
    });
    server.on("/update", HTTP_POST,
        []() {
            server.sendHeader("Connection", "close");
            server.send_P(200, "text/html", UPDATE_OK_HTML);
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
        server.send(200, "text/plain", String(logBuf));
    });
    server.on("/logs.html", HTTP_GET, []() {
        server.send_P(200, "text/html", LOGS_HTML);
    });
    server.on("/status", HTTP_GET, handleStatusJSON);
    server.on("/debug", HTTP_GET, []() {
        String type   = server.arg("type");
        String enable = server.arg("enable");
        String msg;
        if      (type == "nmea")  { dbgNMEA  = (enable=="1"); msg = "NMEA debug "  + String(dbgNMEA  ? "ON":"OFF"); }
        else if (type == "rs485") { dbgRS485 = (enable=="1"); msg = "RS485 debug " + String(dbgRS485 ? "ON":"OFF"); }
        else                      { msg = "unknown"; }
        server.send(200, "text/plain", msg);
    });
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();

    // CAN — non-fatal if bus not present on bench
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        logMsg("WARNING: CAN init failed\n");
    }

    logMsg("=== Gateway %s ready ===\n", SW_VERSION_STRING);
}


// ═════════════════════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    handleRS485();        // always first
    drainCAN();
    server.handleClient();
}
