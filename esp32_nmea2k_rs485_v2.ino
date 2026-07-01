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
 * Version: 2.6.0  (2026-06-28)
 */

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.12.0"
#define SW_BUILD_DATE      "2026-07-01"

// ── WiFi AP ───────────────────────────────────────────────────────────────────
const char *ssid     = "nmea2k_rs485_gw";
const char *password = "123456789";

WebServer server(80);

// ── CAN (NMEA 2000) ───────────────────────────────────────────────────────────
#define CAN_TX_PIN     5
#define CAN_RX_PIN     4
#define CAN_SPEED_KBPS 250
#define CAN_RETRY_MS   5000   // reattempt failed CAN init every 5 s

// ── RS485 (MAX485) ────────────────────────────────────────────────────────────
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 21
#define RS485_BAUDRATE  76800

HardwareSerial RS485Serial(2);

// ── RS485 protocol ────────────────────────────────────────────────────────────
#define FRAME_MAX_LEN    32   // largest MMDC frame we'll accept
                          // observed: 19-byte display frames (Temp/Oil/Fuel) when engine running
                          // MUST be ≥19; 32 gives comfortable headroom
#define RESPONSE_LEN     13
#define MSG_TYPE_REQUEST 0x04  // length byte value for a 4-byte depth-poll frame
#define MSG_CMD_DEPTH    0x09
#define MSG_CMD_PING     0x06  // 4-byte cmd=0x06 frame — appears every MMDC cycle before 0x09
                           // hypothesis: responding to this refreshes the display timer

// ── Depth state ───────────────────────────────────────────────────────────────
#define DEPTH_STALE_MS   15000  // 5 s was too tight; transducer/TWAI gaps up to ~10 s are normal

static uint16_t depthTenths    = 0;  // current depth in tenths of a foot
static uint16_t lastGoodTenths = 0;  // last valid depth, held when stale
static bool     depthValid     = false;
static bool     everHadDepth   = false; // true once we've received at least one valid N2k frame
static uint32_t lastDepthMs    = 0;
static uint8_t  toggleBit      = 0;

// ── Stats ─────────────────────────────────────────────────────────────────────
static uint32_t statDepthRx      = 0;
static uint32_t statRS485Req       = 0;  // depth poll (cmd=0x09) responses sent
static uint32_t statRS485PingResp  = 0;  // cmd=0x06 ping responses sent
static uint32_t statRS485CrcFail   = 0;  // true checksum failures (wire noise / framing)
static uint32_t statRS485Unknown   = 0;  // valid frame, unrecognised command type
static uint32_t statRS485StaleResp = 0;  // responses sent with stale/no N2k data
static uint32_t lastDepthRxMs      = 0;
static uint32_t lastRS485ReqMs     = 0;
static uint32_t lastRS485PingMs    = 0;
static uint32_t lastRS485TxMs      = 0;
static uint8_t  lastCrcFailBuf[4]  = {0, 0, 0, 0};  // last 4-byte frame that failed CRC (debug)

// ── Poll interval tracking ─────────────────────────────────────────────────────
// Measures actual MMDC depth-poll (cmd=0x09) inter-arrival time.
// Helps diagnose display-timer vs poll-rate mismatches.
static uint32_t pollIntervalMin  = 0xFFFFFFFF;  // ms, reset to 0xFFFFFFFF at boot
static uint32_t pollIntervalMax  = 0;
static uint32_t pollIntervalSum  = 0;           // for running average
static uint32_t pollIntervalCnt  = 0;
static uint32_t lastPollArrivalMs = 0;

// ── CAN state ─────────────────────────────────────────────────────────────────
static bool     canReady       = false;
static uint32_t lastCanRetryMs = 0;
static uint32_t statCanRetries = 0;

// ── Raw RS485 RX sniffer — ring buffer of last 128 bytes seen on the bus ──────
// Every byte received from RS485 (before framing/checksum) is captured here so
// we can inspect exactly what the MMDC is sending via the /status page.
#define RAW_RX_BUF_LEN 128  // 128 bytes captures ~10 full 12-byte MMDC cycles
static uint8_t  rawRxBuf[RAW_RX_BUF_LEN];
static uint8_t  rawRxHead  = 0;   // next write slot (wraps)
static uint32_t rawRxTotal = 0;   // total bytes ever captured


// ═════════════════════════════════════════════════════════════════════════════
// Helpers
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

// Append one byte to the raw RX ring buffer (called for every byte read)
static void rawCapture(uint8_t b) {
    rawRxBuf[rawRxHead] = b;
    rawRxHead = (rawRxHead + 1) % RAW_RX_BUF_LEN;
    rawRxTotal++;
}

// Return the last CRC-failing frame as a hex string (for /status debugging)
static String lastCrcFailHex() {
    char out[9];
    snprintf(out, sizeof(out), "%02X%02X%02X%02X",
             lastCrcFailBuf[0], lastCrcFailBuf[1],
             lastCrcFailBuf[2], lastCrcFailBuf[3]);
    return String(out);
}

// Return the ring buffer as a hex string in chronological order
static String rawRxHex() {
    String  out;
    out.reserve(RAW_RX_BUF_LEN * 2 + 4);
    uint32_t count = (rawRxTotal < RAW_RX_BUF_LEN) ? rawRxTotal : RAW_RX_BUF_LEN;
    uint8_t  start = (rawRxTotal < RAW_RX_BUF_LEN) ? 0 : rawRxHead;
    char     hex[3];
    for (uint32_t i = 0; i < count; i++) {
        snprintf(hex, sizeof(hex), "%02X", rawRxBuf[(start + i) % RAW_RX_BUF_LEN]);
        out += hex;
    }
    return out;
}


// ═════════════════════════════════════════════════════════════════════════════
// RS485
// ═════════════════════════════════════════════════════════════════════════════

static inline void rs485Tx() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(RS485_DE_RE_PIN, LOW);  }

// Send our 13-byte depth response on RS485.
// Used for both cmd=0x09 (depth poll) and cmd=0x06 (ping/keepalive).
static void sendDepthResponse() {
    uint32_t now  = millis();
    // "acquired stale" = we had data but lost it (N2k went quiet mid-session)
    // "boot stale"     = we never had data yet this session
    bool acquiredStale = everHadDepth && ((now - lastDepthMs) > DEPTH_STALE_MS);
    bool fresh         = everHadDepth && !acquiredStale;

    // What depth value to send:
    //   fresh           → current reading from N2k
    //   acquired stale  → last good reading (holds the display on dash)
    //   boot stale      → 0.0 ft (nothing yet; show something rather than blank)
    uint16_t d = fresh ? depthTenths : lastGoodTenths;

    // Toggle behaviour: ALWAYS ALTERNATE.
    // We previously froze the toggle on acquired-stale, thinking the MMDC would
    // blink or indicate "old data". In practice it just BLANKS the DEPTH field
    // entirely. A stale-but-visible depth reading is far more useful than a blank.
    // Use statRS485StaleResp to track when we're serving stale data for diagnostics.
    if (!fresh) statRS485StaleResp++;

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
    toggleBit ^= 1;  // always alternate — frozen toggle = MMDC blanks DEPTH entirely
    statRS485Req++;  // caller may override — see sendPingResponse()
    lastRS485TxMs = millis();

    rs485Tx();
    delayMicroseconds(50);     // was 200 µs — reduced to respond within MMDC's RX window.
                                // MAX485 DE propagation <1 µs; 50 µs is plenty to settle.
                                // 200 µs was causing the response to arrive after the MMDC
                                // had already timed out and resumed broadcasting, which caused
                                // collision: toggle advanced on ESP32 but MMDC never saw it,
                                // making every other response appear frozen → "No Response".
    RS485Serial.write(resp, RESPONSE_LEN);
    RS485Serial.flush();       // wait for TX buffer to drain into UART shift register
    delayMicroseconds(300);    // wait for the last byte's stop bit to fully clock out
                                // (76800 baud → 1 byte ≈ 130 µs; 300 µs gives full margin)
    rs485Rx();
}

// Ping response: same depth frame as poll response, but tracked separately.
// cmd=0x06 appears every MMDC cycle just before the cmd=0x09 depth poll.
// Hypothesis: responding to it refreshes the MMDC display timer so depth
// stays visible between the slower (~2s) depth polls.
static void sendPingResponse() {
    // Reuse sendDepthResponse() for the frame — it already handles stale/fresh
    // depth and always alternates the toggle.
    // Undo the statRS485Req++ it adds (ping has its own counter).
    sendDepthResponse();
    statRS485Req--;        // un-count from depth-poll stat
    statRS485PingResp++;   // count as ping instead
    lastRS485PingMs = millis();
}

static void handleRS485() {
    static uint8_t buf[FRAME_MAX_LEN];
    static uint8_t idx      = 0;
    static uint8_t expected = 0;

    while (RS485Serial.available()) {
        uint8_t b = RS485Serial.read();
        rawCapture(b);   // ← capture every byte before framing logic

        if (idx == 0) {
            // First byte is the MMDC frame length (total bytes incl. length + checksum).
            // Accept 4–FRAME_MAX_LEN; anything outside that range is skipped.
            // Using FRAME_MAX_LEN (not 4) is critical: the MMDC sends 5-, 8-, 12-,
            // and 13-byte status broadcasts that contain 0x04 as a *data* byte.
            // When we only accepted length=4, those 0x04 data bytes were mistaken
            // for frame-start bytes, generating spurious CRC failures.
            // Accepting the true frame length consumes the whole message cleanly.
            if (b < 4 || b > FRAME_MAX_LEN) continue;
            expected = b;
        }
        buf[idx++] = b;
        if (idx < expected) continue;

        // Full frame received — verify checksum then dispatch.
        if (!verifyChecksum(buf, expected)) {
            memcpy(lastCrcFailBuf, buf, 4);  // capture first 4 bytes for /status debug
            statRS485CrcFail++;              // genuine wire error on a known-length frame
        } else if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_DEPTH) {
            // 4-byte depth poll (cmd=0x09) from MMDC — reply with current depth.
            // Track inter-poll interval for display-timer diagnosis.
            uint32_t now = millis();
            if (lastPollArrivalMs > 0) {
                uint32_t interval = now - lastPollArrivalMs;
                if (interval < pollIntervalMin) pollIntervalMin = interval;
                if (interval > pollIntervalMax) pollIntervalMax = interval;
                pollIntervalSum += interval;
                pollIntervalCnt++;
            }
            lastPollArrivalMs = now;
            lastRS485ReqMs    = now;
            sendDepthResponse();
        } else if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_PING) {
            // 4-byte cmd=0x06 frame — appears each MMDC cycle before the depth poll.
            // Respond with our depth frame to try to keep the display timer alive.
            sendPingResponse();
        } else {
            // Valid frame, not a recognised command — MMDC status broadcast, ignore.
            statRS485Unknown++;
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
    // PGN 128267 depth field is 32-bit (bytes 1–4), resolution 0.01 m.
    // Reading only 3 bytes missed the MSB and let N2K error/not-available
    // codes (0xFFFFFFFF, etc.) pass the sanity check as huge valid depths.
    // Those bogus values were stored in lastGoodTenths, then sent as signed
    // 16-bit overflow (e.g. 0xFD50 = −688) which the MMDC clamps to 0 and blinks.
    uint32_t rawM = (uint32_t)f.data[1] | ((uint32_t)f.data[2] << 8) |
                    ((uint32_t)f.data[3] << 16) | ((uint32_t)f.data[4] << 24);
    // Reject N2K “not available” (0xFFFFFFFF / 0xFFFFFFFE) and anything
    // deeper than 300 m (30 000 raw units ≈ 984 ft) — clearly out of water.
    if (rawM > 30000) return;
    float depthM  = rawM * 0.01f;
    float depthFt = depthM * 3.28084f;
    depthTenths    = (uint16_t)(depthFt * 10.0f + 0.5f);
    lastGoodTenths = depthTenths;
    depthValid     = true;
    everHadDepth   = true;
    lastDepthMs    = millis();
    lastDepthRxMs  = millis();
    statDepthRx++;
}

// Retry CAN init every CAN_RETRY_MS if it failed at boot.
// On NMEA 2000 networks the bus may not be stable within the first second
// of power-on, so the initial begin() can fail even though hardware is fine.
static void maybeRetryCAN() {
    if (canReady) return;
    uint32_t now = millis();
    if (now - lastCanRetryMs < CAN_RETRY_MS) return;
    lastCanRetryMs = now;
    statCanRetries++;
    if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        Serial.printf("CAN init OK on retry #%lu\n", statCanRetries);
        canReady = true;
    } else {
        Serial.printf("CAN retry #%lu failed — will retry in %ds\n",
                      statCanRetries, CAN_RETRY_MS / 1000);
    }
}

static void drainCAN() {
    if (!canReady) return;
    // Drain up to 8 frames per call — avoids TWAI FIFO overflow when
    // the loop stalls briefly on RS485/HTTP, without the unbounded while
    // that caused hangs with large RX queues in the ESP32-TWAI-CAN library.
    for (uint8_t i = 0; i < 8; i++) {
        CanFrame f;
        if (!ESP32Can.readFrame(f)) break;
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
    const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>NMEA2k RS485 Gateway</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;}
    .card{background:#23272a;border-radius:12px;padding:2em 2.5em;box-shadow:0 2px 16px #000a;min-width:340px;max-width:500px;width:100%;}
    h2{color:#90caf9;margin-top:0;}
    table{width:100%;border-collapse:collapse;table-layout:fixed;}
    td{padding:8px 4px;border-bottom:1px solid #333;vertical-align:top;overflow-wrap:break-word;word-break:break-word;}
    td:first-child{color:#90caf9;width:48%;}
    .ok{color:#b0f2bc;} .err{color:#ef5350;} .warn{color:#ffcc80;}
    .hex{font-family:monospace;font-size:0.8em;word-break:break-all;line-height:1.6;}
    a{color:#90caf9;}
  </style>
</head>
<body>
<div class='card'>
  <h2>NMEA2k RS485 Gateway</h2>
  <table>
    <tr><td>Firmware</td><td id='fw'>-</td></tr>
    <tr><td>Uptime</td><td id='up'>-</td></tr>
    <tr><td>CAN (NMEA2000)</td><td id='can'>-</td></tr>
    <tr><td>CAN retry count</td><td id='canr'>-</td></tr>
    <tr><td>Depth</td><td id='depth'>-</td></tr>
    <tr><td>Depth state</td><td id='dstate'>-</td></tr>
    <tr><td>N2k depth frames</td><td id='drx'>-</td></tr>
    <tr><td>Last N2k depth</td><td id='dage'>-</td></tr>
    <tr><td>RS485 depth polls (0x09)</td><td id='req'>-</td></tr>
    <tr><td>RS485 ping resp (0x06)</td><td id='pingresp'>-</td></tr>
    <tr><td>Poll interval min/avg/max</td><td id='pollint'>-</td></tr>
    <tr><td>RS485 CRC failures</td><td id='crcfail'>-</td></tr>
    <tr><td>Last CRC fail frame</td><td id='crcfailhex' class='hex'>-</td></tr>
    <tr><td>RS485 unknown frames</td><td id='unknown'>-</td></tr>
    <tr><td>RS485 stale replies</td><td id='staleresp'>-</td></tr>
    <tr><td>Last depth poll</td><td id='reqage'>-</td></tr>
    <tr><td>Last ping (0x06)</td><td id='pingage'>-</td></tr>
    <tr><td>Last RS485 reply</td><td id='txage'>-</td></tr>
    <tr><td>Raw RX total</td><td id='rawtotal'>-</td></tr>
    <tr>
      <td colspan='2'>
        <span style='color:#90caf9'>Raw RS485 RX (last 128 bytes)</span><br>
        <span class='hex' id='raw'>-</span>
      </td>
    </tr>
  </table>
  <br><a href='/'>Firmware Update</a>
</div>
<script>
function age(ms){ return ms < 0 ? 'never' : ms + 'ms ago'; }
function update(){
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('fw').textContent = d.firmware;
    document.getElementById('up').textContent = d.uptime_s + 's';
    var can = document.getElementById('can');
    can.textContent = d.can_ready ? 'ready' : 'NOT READY';
    can.className = d.can_ready ? 'ok' : 'err';
    document.getElementById('canr').textContent = d.can_retries;
    document.getElementById('depth').textContent = d.depth_ft + ' ft';
    var el = document.getElementById('dstate');
    el.textContent = d.depth_state;
    el.className = d.depth_state === 'fresh' ? 'ok' : (d.depth_state === 'boot_no_data' ? 'warn' : 'err');
    document.getElementById('drx').textContent = d.depth_rx;
    document.getElementById('dage').textContent = age(d.depth_age_ms);
    document.getElementById('req').textContent = d.rs485_req;
    document.getElementById('pingresp').textContent = d.rs485_ping_resp;
    var pi = d.poll_interval_cnt > 0
      ? d.poll_interval_min_ms + '/' + Math.round(d.poll_interval_avg_ms) + '/' + d.poll_interval_max_ms + ' ms (' + d.poll_interval_cnt + ' samples)'
      : '(no data yet)';
    document.getElementById('pollint').textContent = pi;
    var crcEl = document.getElementById('crcfail');
    crcEl.textContent = d.rs485_crc_fail;
    crcEl.className = d.rs485_crc_fail > 0 ? 'err' : '';
    document.getElementById('crcfailhex').textContent = d.rs485_last_crc_fail_hex || '-';
    var unkEl = document.getElementById('unknown');
    unkEl.textContent = d.rs485_unknown;
    unkEl.className = d.rs485_unknown > 0 ? 'warn' : '';
    var staleEl = document.getElementById('staleresp');
    staleEl.textContent = d.rs485_stale_resp;
    staleEl.className = d.rs485_stale_resp > 0 ? 'warn' : '';
    document.getElementById('reqage').textContent = age(d.rs485_req_age_ms);
    document.getElementById('pingage').textContent = age(d.rs485_ping_age_ms);
    document.getElementById('txage').textContent = age(d.rs485_tx_age_ms);
    document.getElementById('rawtotal').textContent = d.raw_rx_total + ' bytes';
    // format hex in groups of 4 bytes (8 hex chars) for readability
    var h = d.raw_rx_hex;
    var out = '';
    for (var i = 0; i < h.length; i += 8) { out += h.substr(i, 8) + ' '; }
    document.getElementById('raw').textContent = out.trim() || '(none yet)';
  }).catch(()=>{});
}
update();
setInterval(update, 1000);
</script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleData() {
    uint32_t now           = millis();
    bool acquiredStale     = everHadDepth && ((now - lastDepthMs) > DEPTH_STALE_MS);
    bool fresh             = everHadDepth && !acquiredStale;
    // depth_state: "fresh" | "acquired_stale" | "boot_no_data"
    const char* depthState = fresh ? "fresh" : (acquiredStale ? "acquired_stale" : "boot_no_data");
    String json = "{";
    json += "\"firmware\":\""      + String(SW_VERSION_STRING) + "\",";
    json += "\"build_date\":\""    + String(SW_BUILD_DATE)     + "\",";
    json += "\"uptime_s\":"        + String(now / 1000)        + ",";
    json += "\"can_ready\":"       + String(canReady ? "true" : "false") + ",";
    json += "\"can_retries\":"     + String(statCanRetries)    + ",";
    json += "\"depth_ft\":"        + String(depthTenths / 10.0f, 1) + ",";
    json += "\"depth_state\":\""   + String(depthState)        + "\",";
    json += "\"depth_valid\":"     + String(fresh ? "true" : "false") + ",";
    json += "\"ever_had_depth\":"  + String(everHadDepth ? "true" : "false") + ",";
    json += "\"depth_rx\":"        + String(statDepthRx)       + ",";
    json += "\"depth_age_ms\":"    + String(lastDepthRxMs ? (int32_t)(now - lastDepthRxMs) : -1) + ",";
    json += "\"rs485_req\":"          + String(statRS485Req)       + ",";
    json += "\"rs485_ping_resp\":"     + String(statRS485PingResp)  + ",";
    json += "\"rs485_req_age_ms\":"    + String(lastRS485ReqMs  ? (int32_t)(now - lastRS485ReqMs)  : -1) + ",";
    json += "\"rs485_ping_age_ms\":"   + String(lastRS485PingMs ? (int32_t)(now - lastRS485PingMs) : -1) + ",";
    json += "\"rs485_tx_age_ms\":"     + String(lastRS485TxMs   ? (int32_t)(now - lastRS485TxMs)   : -1) + ",";
    json += "\"rs485_crc_fail\":"      + String(statRS485CrcFail)   + ",";
    json += "\"rs485_unknown\":"       + String(statRS485Unknown)    + ",";
    json += "\"rs485_last_crc_fail_hex\":\"" + lastCrcFailHex() + "\",";
    json += "\"rs485_stale_resp\":"    + String(statRS485StaleResp) + ",";
    json += "\"poll_interval_cnt\":"   + String(pollIntervalCnt)    + ",";
    json += "\"poll_interval_min_ms\":" + String(pollIntervalCnt ? pollIntervalMin : 0) + ",";
    json += "\"poll_interval_max_ms\":" + String(pollIntervalMax)   + ",";
    json += "\"poll_interval_avg_ms\":" + String(pollIntervalCnt ? (float)pollIntervalSum/pollIntervalCnt : 0.0f, 1) + ",";
    json += "\"raw_rx_total\":"  + String(rawRxTotal)        + ",";
    json += "\"raw_rx_hex\":\""  + rawRxHex()                + "\"";
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

    // Web routes
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

    // CAN — non-fatal at boot; retried every CAN_RETRY_MS in loop()
    // NMEA 2000 networks sometimes need a few seconds to stabilise after
    // power-on, so begin() may fail even when hardware is physically fine.
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        Serial.printf("WARNING: CAN init failed — will retry every %ds\n",
                      CAN_RETRY_MS / 1000);
        canReady = false;
    } else {
        Serial.printf("NMEA2000 CAN ready %dkbps\n", CAN_SPEED_KBPS);
        canReady = true;
    }

    Serial.println("Setup complete.");
}


// ═════════════════════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    handleRS485();    // first, always — UART HW buffer holds bytes safely
    maybeRetryCAN();  // no-op when canReady; retries CAN init every 5 s if needed
    drainCAN();
    server.handleClient();
}
