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
 * Author : caseyn
 * Version: 2.3.0  (2026-04-16)
 */

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.3.5"
#define SW_BUILD_DATE      "2026-04-16"

// ── WiFi AP ───────────────────────────────────────────────────────────────────
const char *ssid     = "nmea2k_rs485_gw";
const char *password = "123456789";

WebServer server(80);

// ── CAN (NMEA 2000) ───────────────────────────────────────────────────────────
#define CAN_TX_PIN     5
#define CAN_RX_PIN     4
#define CAN_SPEED_KBPS 250

// ── RS485 (MAX485) ────────────────────────────────────────────────────────────
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 21
#define RS485_BAUDRATE  76800

HardwareSerial RS485Serial(2);

// ── RS485 protocol ────────────────────────────────────────────────────────────
#define REQUEST_LEN      4
#define RESPONSE_LEN     13
#define MSG_TYPE_REQUEST 0x04
#define MSG_CMD_DEPTH    0x09

// ── Depth state ───────────────────────────────────────────────────────────────
#define DEPTH_STALE_MS   5000

static uint16_t depthTenths = 0;
static bool     depthValid  = false;
static uint32_t lastDepthMs = 0;
static uint8_t  toggleBit   = 0;

// ── Stats ─────────────────────────────────────────────────────────────────────
static uint32_t statDepthRx   = 0;
static uint32_t statRS485Req  = 0;
static uint32_t statRS485Bad  = 0;
static uint32_t lastDepthRxMs = 0;
static uint32_t lastRS485ReqMs= 0;
static uint32_t lastRS485TxMs = 0;

// ── CAN ready flag — only set if ESP32Can.begin() succeeds ───────────────────
static bool canReady = false;


// ═════════════════════════════════════════════════════════════════════════════
// Checksum — 8-bit two's complement
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
    lastRS485TxMs = millis();

    rs485Tx();
    delayMicroseconds(200);
    RS485Serial.write(resp, RESPONSE_LEN);
    RS485Serial.flush();
    delayMicroseconds(100);
    rs485Rx();
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

        if (verifyChecksum(buf, expected) &&
            buf[0] == MSG_TYPE_REQUEST &&
            buf[1] == MSG_CMD_DEPTH) {
            lastRS485ReqMs = millis();
            sendDepthResponse();
        } else {
            statRS485Bad++;
        }
        idx = expected = 0;
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// NMEA 2000 / CAN
// ═════════════════════════════════════════════════════════════════════════════

static uint32_t extractPGN(uint32_t id) {
    uint8_t dp = (id >> 24) & 0x01;
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >>  8) & 0xFF;
    return (pf < 240) ? ((uint32_t)dp<<16)|((uint32_t)pf<<8)
                      : ((uint32_t)dp<<16)|((uint32_t)pf<<8)|ps;
}

static void handleDepth(const CanFrame &f) {
    if (f.data_length_code < 7) return;
    uint32_t rawM   = (uint32_t)f.data[1] | ((uint32_t)f.data[2]<<8) | ((uint32_t)f.data[3]<<16);
    float    depthM = rawM * 0.01f;
    float    depthFt= depthM * 3.28084f;
    depthTenths = (uint16_t)(depthFt * 10.0f + 0.5f);
    depthValid  = true;
    lastDepthMs = millis();
    lastDepthRxMs = millis();
    statDepthRx++;
}

static void drainCAN() {
    if (!canReady) return;
    // Single read per loop iteration — matches v1 behaviour.
    // while() loop caused hangs with ESP32-TWAI-CAN library.
    CanFrame f;
    if (ESP32Can.readFrame(f)) {
        if (extractPGN(f.identifier) == 128267) handleDepth(f);
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// Web server
// ═════════════════════════════════════════════════════════════════════════════

const char *ota_upload_form = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>NMEA2k RS485 Gateway</title>
    <style>
      body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;margin:0;padding:0;min-height:100vh;display:flex;flex-direction:column;align-items:center;justify-content:center;}
      h2{color:#90caf9;}
      form{background:#23272a;padding:2em 2.5em;border-radius:12px;box-shadow:0 2px 16px #000a;display:flex;flex-direction:column;gap:1em;align-items:center;}
      input[type='file']{color:#f1f1f1;background:#181a1b;border:1px solid #444;border-radius:6px;padding:0.5em;}
      input[type='submit']{background:#1976d2;color:#f1f1f1;padding:0.7em 2em;border:none;border-radius:6px;font-size:1em;cursor:pointer;}
      input[type='submit']:hover{background:#1565c0;}
      a{color:#90caf9;margin-top:1em;}
    </style>
  </head>
  <body>
    <h2>NMEA2k RS485 Gateway</h2>
    <form method='POST' action='/update' enctype='multipart/form-data'>
      <input type='file' name='update'>
      <input type='submit' value='Upload Firmware'>
    </form>
    <a href='/status'>Status</a>
  </body>
</html>
)rawliteral";

const char *update_success_html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>Update Success</title>
    <meta http-equiv="refresh" content="5; url=/" />
    <style>
      body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;margin:0;display:flex;align-items:center;justify-content:center;min-height:100vh;}
      .box{background:#23272a;padding:2em 2.5em;border-radius:12px;text-align:center;}
      h2{color:#90caf9;}p{color:#b0f2bc;}
    </style>
  </head>
  <body>
    <div class="box"><h2>Update Success!</h2><p>Rebooting...</p></div>
  </body>
</html>
)rawliteral";

void handleStatus() {
    bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
    const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>NMEA2k RS485 Gateway</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;}
    .card{background:#23272a;border-radius:12px;padding:2em 2.5em;box-shadow:0 2px 16px #000a;min-width:320px;}
    h2{color:#90caf9;margin-top:0;}
    table{width:100%;border-collapse:collapse;}
    td{padding:8px 4px;border-bottom:1px solid #333;}
    td:first-child{color:#90caf9;width:50%;}
    .stale{color:#ef5350;}
    a{color:#90caf9;}
  </style>
</head>
<body>
<div class='card'>
  <h2>NMEA2k RS485 Gateway</h2>
  <table>
    <tr><td>Firmware</td><td id='fw'>-</td></tr>
    <tr><td>Uptime</td><td id='up'>-</td></tr>
    <tr><td>Depth</td><td id='depth'>-</td></tr>
    <tr><td>Depth valid</td><td id='valid'>-</td></tr>
    <tr><td>CAN depth frames</td><td id='drx'>-</td></tr>
    <tr><td>Last CAN depth</td><td id='dage'>-</td></tr>
    <tr><td>RS485 requests</td><td id='req'>-</td></tr>
    <tr><td>Last RS485 request</td><td id='reqage'>-</td></tr>
    <tr><td>Last RS485 reply</td><td id='txage'>-</td></tr>
    <tr><td>RS485 bad</td><td id='bad'>-</td></tr>
  </table>
  <br><a href='/'>Firmware Update</a>
</div>
<script>
function update(){
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('fw').textContent=d.firmware;
    document.getElementById('up').textContent=d.uptime_s+'s';
    document.getElementById('depth').textContent=d.depth_ft+' ft';
    var el=document.getElementById('valid');
    el.textContent=d.depth_valid?'yes':'no (stale)';
    el.className=d.depth_valid?'':'stale';
    document.getElementById('drx').textContent=d.depth_rx;
    document.getElementById('dage').textContent=d.depth_age_ms<0?'never':d.depth_age_ms+'ms ago';
    document.getElementById('req').textContent=d.rs485_req;
    document.getElementById('reqage').textContent=d.rs485_req_age_ms<0?'never':d.rs485_req_age_ms+'ms ago';
    document.getElementById('txage').textContent=d.rs485_tx_age_ms<0?'never':d.rs485_tx_age_ms+'ms ago';
    document.getElementById('bad').textContent=d.rs485_bad;
  }).catch(()=>{});
}
update(); setInterval(update,1000);
</script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleData() {
    bool stale = !depthValid || ((millis() - lastDepthMs) > DEPTH_STALE_MS);
    uint32_t now = millis();
    String json = "{";
    json += "\"firmware\":\"" + String(SW_VERSION_STRING) + "\",";
    json += "\"build_date\":\"" + String(SW_BUILD_DATE) + "\",";
    json += "\"uptime_s\":" + String(now / 1000) + ",";
    json += "\"depth_ft\":" + String(depthTenths / 10.0f, 1) + ",";
    json += "\"depth_valid\":" + String(stale ? "false" : "true") + ",";
    json += "\"depth_rx\":" + String(statDepthRx) + ",";
    json += "\"depth_age_ms\":" + String(lastDepthRxMs ? (int32_t)(now - lastDepthRxMs) : -1) + ",";
    json += "\"rs485_req\":" + String(statRS485Req) + ",";
    json += "\"rs485_req_age_ms\":" + String(lastRS485ReqMs ? (int32_t)(now - lastRS485ReqMs) : -1) + ",";
    json += "\"rs485_tx_age_ms\":" + String(lastRS485TxMs ? (int32_t)(now - lastRS485TxMs) : -1) + ",";
    json += "\"rs485_bad\":" + String(statRS485Bad);
    json += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
}


// ═════════════════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n--- esp32 NMEA2k RS485 Gateway ---");
    Serial.printf("FW Version: %s (Build: %s)\n", SW_VERSION_STRING, SW_BUILD_DATE);

    // RS485
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    rs485Rx();
    RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    Serial.println("RS485 initialized");

    // WiFi AP
    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Web routes — same pattern as mefi project
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", ota_upload_form);
    });
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/data",   HTTP_GET, handleData);
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", update_success_html);
        delay(1000);
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if      (upload.status == UPLOAD_FILE_START) { Update.begin(UPDATE_SIZE_UNKNOWN); }
        else if (upload.status == UPLOAD_FILE_WRITE) { Update.write(upload.buf, upload.currentSize); }
        else if (upload.status == UPLOAD_FILE_END)   { Update.end(true); }
    });

    server.begin();
    Serial.println("HTTP server started");

    // CAN — non-fatal if not connected on bench
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        Serial.println("WARNING: CAN init failed - no NMEA2000 input");
        canReady = false;
    } else {
        Serial.printf("NMEA2000 CAN ready %dkbps\n", CAN_SPEED_KBPS);
        canReady = true;
    }

    Serial.println("Setup complete.");
}


// ═════════════════════════════════════════════════════════════════════════════
// loop() — same pattern as mefi project
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    handleRS485();
    drainCAN();
    server.handleClient();
}
