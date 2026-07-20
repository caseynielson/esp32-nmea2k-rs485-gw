/*
 * esp32 NMEA 2000 → RS485 Gateway
 *
 * Bridges NMEA 2000 water depth (PGN 128267) to the Medallion MMDC
 * proprietary RS485 protocol.
 *
 * Hardware
 * ────────
 *  CAN  : TJA1050 - TX→GPIO5, RX→GPIO4, 250 kbps
 *  RS485: MAX485  - TX→GPIO17, RX→GPIO16, DE/RE→GPIO21, 76800 baud
 *  WiFi : Soft-AP  SSID "nmea2k_rs485_gw" / pw "123456789"
 *
 * Author : caseyn
 * Version: 2.15.1  (2026-07-02)
 */

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define SW_VERSION_STRING  "v2.19.2"
#define SW_BUILD_DATE      "2026-07-20"

// ── WiFi AP ───────────────────────────────────────────────────────────────────
const char *ssid     = "nmea2k_rs485_gw";
const char *password = "123456789";

WebServer server(80);

// ── FreeRTOS: RS485 task (high priority) + loop() (lower priority), both Core 1 ─────
// Arduino loop() runs on Core 1 at priority 1. rs485Task also runs on Core 1
// at priority 2, so it PREEMPTS loop() whenever RS485 bytes arrive.
// HTTP/CAN work in loop() cannot block the RS485 response path.
// The MMDC polls every ~2s; any loop() blockage >~20ms could cause a missed response.
SemaphoreHandle_t depthMutex;  // protects shared depth state between cores

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

// ── Runtime-tunable parameters (adjustable via /tune without reflashing) ────────
// These are the factory defaults; changed at runtime via POST /tune.
struct TuneParams {
    uint16_t staleMs;       // ms before depth is considered stale
    uint16_t preTxDelayUs;  // μs DE asserted before first byte
    uint8_t  freshByte11;   // response byte[11] when depth is FRESH
    uint8_t  staleByte11;   // response byte[11] when depth is STALE
    bool     pingEnabled;   // whether to respond to cmd=0x06 ping frames
    bool     bootFallback;  // send 0.0ft response when no N2k depth ever received
    // Mystery bytes — tunable for reverse-engineering MMDC blink behavior
    uint8_t  b2;            // byte[2]  default 0x14
    uint8_t  b3;            // byte[3]  default 0xAA
    uint8_t  b6;            // byte[6]  default 0xFF
    uint8_t  b7;            // byte[7]  default 0x03
    uint8_t  b8;            // byte[8]  default 0xFF
    uint8_t  b9;            // byte[9]  default 0x03
    bool     overrideDepth; // when true, send depthOverride instead of real depth
    uint16_t depthOverride; // raw tenths-of-foot value to send (e.g. 0xFFFF)
    bool     freezeToggle;  // when true, stop alternating toggleBit (test: does MMDC blink?)
} tune = {
    .staleMs       = 5000,
    .preTxDelayUs  = 50,
    .freshByte11   = 0x02,
    .staleByte11   = 0x02,
    .pingEnabled   = false,
    .bootFallback  = true,
    .b2            = 0x14,
    .b3            = 0xAA,
    .b6            = 0xFF,
    .b7            = 0x03,
    .b8            = 0xFF,
    .b9            = 0x03,
    .overrideDepth = false,
    .depthOverride = 0xFFFF,
    .freezeToggle  = false,
};

// ── RS485 protocol ────────────────────────────────────────────────────────────
#define FRAME_MAX_LEN    32   // largest MMDC frame we'll accept
                          // observed: 19-byte display frames (Temp/Oil/Fuel) when engine running
                          // MUST be ≥19; 32 gives comfortable headroom
#define RESPONSE_LEN     13
#define MSG_TYPE_REQUEST 0x04  // length byte value for a 4-byte depth-poll frame
#define MSG_CMD_DEPTH    0x09
#define MSG_CMD_PING     0x06  // 4-byte cmd=0x06 frame - appears every MMDC cycle before 0x09
                           // hypothesis: responding to this refreshes the display timer

// ── Depth state ───────────────────────────────────────────────────────────────
// stale threshold is now runtime-tunable via tune.staleMs
#define DEPTH_STALE_MS_DEFAULT 5000

// Shared between Core 0 (CAN/N2k writer) and Core 1 (RS485 reader).
// All reads/writes in handleDepth() and sendDepthResponse() must hold depthMutex.
static volatile uint16_t depthTenths    = 0;  // current depth in tenths of a foot
static volatile uint16_t lastGoodTenths = 0;  // last valid depth, held when stale
static volatile bool     depthValid     = false;
static volatile bool     everHadDepth   = false;
static volatile uint32_t lastDepthMs    = 0;
static volatile uint8_t  toggleBit      = 0;

// ── Stats ─────────────────────────────────────────────────────────────────────
// Stats shared between cores — written by Core 1 (RS485 task), read by Core 0 (HTTP).
// Use volatile; occasional torn reads on HTTP display are acceptable.
static volatile uint32_t statDepthRx      = 0;
static volatile uint32_t statRS485Req       = 0;
static volatile uint32_t statRS485PingResp  = 0;
static volatile uint32_t statRS485CrcFail   = 0;
static volatile uint32_t statRS485Unknown   = 0;
static volatile uint32_t statRS485StaleResp = 0;
static volatile uint32_t lastDepthRxMs      = 0;
static volatile uint32_t lastRS485ReqMs     = 0;
static volatile uint32_t lastRS485PingMs    = 0;
static volatile uint32_t lastRS485TxMs      = 0;
static uint8_t  lastCrcFailBuf[4]  = {0, 0, 0, 0};

// ── Poll interval tracking ─────────────────────────────────────────────────────
// Measures actual MMDC depth-poll (cmd=0x09) inter-arrival time.
// Helps diagnose display-timer vs poll-rate mismatches.
static volatile uint32_t pollIntervalMin  = 0xFFFFFFFF;
static volatile uint32_t pollIntervalMax  = 0;
static volatile uint32_t pollIntervalSum  = 0;
static volatile uint32_t pollIntervalCnt  = 0;
static volatile uint32_t lastPollArrivalMs = 0;

// ── /drop — response suppression for blink-timing experiments ─────────────────
// When dropUntilMs > 0, rs485Task silently discards 0x09 depth-poll responses
// until millis() >= dropUntilMs, then auto-resumes.
// Written by loop() HTTP handler; read by rs485Task (Core 1).
// Atomic enough for a single uint32_t on this architecture.
static volatile uint32_t dropUntilMs   = 0;   // 0 = not dropping
static volatile uint32_t statDropCount = 0;   // polls suppressed since last /drop

// ── CAN state ─────────────────────────────────────────────────────────────────
static bool     canReady       = false;
static uint32_t lastCanRetryMs = 0;
static uint32_t statCanRetries = 0;

// ── Raw RS485 RX sniffer - ring buffer of last 128 bytes seen on the bus ──────
// Every byte received from RS485 (before framing/checksum) is captured here so
// we can inspect exactly what the MMDC is sending via the /status page.
#define RAW_RX_BUF_LEN 256  // 256 bytes captures ~20 full MMDC cycles for better visibility
static uint8_t  rawRxBuf[RAW_RX_BUF_LEN];
static uint16_t rawRxHead  = 0;
static volatile uint32_t rawRxTotal = 0;


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
    // Read shared depth state under mutex (Core 0 writes via handleDepth)
    uint16_t localDepthTenths, localLastGoodTenths;
    bool localEverHad;
    uint32_t localLastDepthMs;
    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        localDepthTenths    = depthTenths;
        localLastGoodTenths = lastGoodTenths;
        localEverHad        = everHadDepth;
        localLastDepthMs    = lastDepthMs;
        xSemaphoreGive(depthMutex);
    } else {
        // Couldn't get mutex in 2ms — use last known values, skip this response
        return;
    }
    bool acquiredStale = localEverHad && ((now - localLastDepthMs) > (uint32_t)tune.staleMs);
    bool fresh         = localEverHad && !acquiredStale;
    bool bootNoData    = !localEverHad;

    // ── /drop suppression check ───────────────────────────────────────────────
    // If a /drop window is active, silently eat this poll and return.
    // The MMDC will time out and (hopefully) blink. Auto-resumes when window expires.
    if (dropUntilMs > 0) {
        if (millis() < dropUntilMs) {
            statDropCount++;
            statRS485Req++;  // count the poll as seen, just not answered
            return;          // no response sent
        } else {
            dropUntilMs = 0; // window expired — resume normal responses
        }
    }

    // What depth value to send:
    //   fresh           → real N2k depth, solid display
    //   acquired stale  → 0xFFFF sentinel: MMDC blinks "400" (lost bottom indicator)
    //   boot no data    → 0.0 ft fallback, or suppress to let MMDC blank
    uint16_t d;
    if (fresh) {
        d = localDepthTenths;
    } else if (bootNoData) {
        if (!tune.bootFallback) {
            statRS485Req++;
            lastRS485TxMs = millis();
            return;
        }
        d = 0; // 0.0 ft fallback at boot
    } else {
        // acquired stale: send 0xFFFF — MMDC recognises this as "no valid depth"
        // and blinks its maximum range value (400 ft = lost bottom indicator).
        // Confirmed 2026-07-20 by experiment.
        d = 0xFFFF;
    }

    uint8_t b11 = tune.freshByte11; // 0x02 = solid; depth value controls blink, not this byte

    if (!fresh) statRS485StaleResp++;

    if (tune.overrideDepth) d = tune.depthOverride;

    uint8_t resp[RESPONSE_LEN] = {
        RESPONSE_LEN,
        MSG_CMD_DEPTH,
        tune.b2, tune.b3,
        (uint8_t)(d & 0xFF), (uint8_t)(d >> 8),
        tune.b6, tune.b7,
        tune.b8, tune.b9,
        toggleBit,
        b11,
        0x00
    };
    resp[12] = calcChecksum(resp, 12);
    if (!tune.freezeToggle) toggleBit ^= 1;  // frozen toggle test: does MMDC blink or blank?
    statRS485Req++;  // caller may override - see sendPingResponse()
    lastRS485TxMs = millis();

    rs485Tx();
    delayMicroseconds(tune.preTxDelayUs);
    RS485Serial.write(resp, RESPONSE_LEN);
    RS485Serial.flush();       // wait for TX buffer to drain into UART shift register
    delayMicroseconds(300);    // wait for the last byte's stop bit to fully clock out
                                // (76800 baud → 1 byte ≈ 130 μs; 300 μs gives full margin)
    rs485Rx();
}

// Ping response: same depth frame as poll response, but tracked separately.
// cmd=0x06 appears every MMDC cycle just before the cmd=0x09 depth poll.
// Hypothesis: responding to it refreshes the MMDC display timer so depth
// stays visible between the slower (~2s) depth polls.
static void sendPingResponse() {
    statRS485PingResp++;   // always count observed ping frames
    lastRS485PingMs = millis();
    if (!tune.pingEnabled) {
        // ping response disabled (default) - MMDC gives no response window after 0x06
        // responding would cause RS485 collision with MMDC's next broadcast frame
        return;
    }
    // Reuse sendDepthResponse() for the frame - it already handles stale/fresh
    // depth and always alternates the toggle.
    // Undo the statRS485Req++ it adds (ping has its own counter).
    sendDepthResponse();
    statRS485Req--;        // un-count from depth-poll stat
    // statRS485PingResp already incremented above; lastRS485PingMs already set
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
            // Accept 4-FRAME_MAX_LEN; anything outside that range is skipped.
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

        // Full frame received - verify checksum then dispatch.
        if (!verifyChecksum(buf, expected)) {
            memcpy(lastCrcFailBuf, buf, 4);  // capture first 4 bytes for /status debug
            statRS485CrcFail++;              // genuine wire error on a known-length frame
        } else if (buf[0] == MSG_TYPE_REQUEST && buf[1] == MSG_CMD_DEPTH) {
            // 4-byte depth poll (cmd=0x09) from MMDC - reply with current depth.
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
            // 4-byte cmd=0x06 frame - appears each MMDC cycle before the depth poll.
            // IMPORTANT: The MMDC does NOT open a response window after this frame.
            // It immediately resumes broadcasting on the shared RS485 bus.
            // Responding causes collisions that corrupt our 0x09 depth response.
            // Only respond if explicitly enabled via /tune (for experimental testing).
            sendPingResponse();
        } else {
            // Valid frame, not a recognised command - MMDC status broadcast, ignore.
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
    uint32_t rawM = (uint32_t)f.data[1] | ((uint32_t)f.data[2] << 8) |
                    ((uint32_t)f.data[3] << 16) | ((uint32_t)f.data[4] << 24);
    if (rawM > 30000) return;
    float depthM  = rawM * 0.01f;
    float depthFt = depthM * 3.28084f;
    uint16_t tenths = (uint16_t)(depthFt * 10.0f + 0.5f);
    // Take mutex: Core 1 RS485 task reads these fields during sendDepthResponse()
    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        depthTenths    = tenths;
        lastGoodTenths = tenths;
        depthValid     = true;
        everHadDepth   = true;
        lastDepthMs    = millis();
        lastDepthRxMs  = millis();
        statDepthRx++;
        xSemaphoreGive(depthMutex);
    }
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
        Serial.printf("CAN retry #%lu failed - will retry in %ds\n",
                      statCanRetries, CAN_RETRY_MS / 1000);
    }
}

static void drainCAN() {
    if (!canReady) return;
    // Drain up to 8 frames per call - avoids TWAI FIFO overflow when
    // the loop stalls briefly on RS485/HTTP, without the unbounded while
    // that caused hangs with large RX queues in the ESP32-TWAI-CAN library.
    for (uint8_t i = 0; i < 8; i++) {
        CanFrame f;
        if (!ESP32Can.readFrame(f)) break;
        if (extractPGN(f.identifier) == 128267) handleDepth(f);
    }
}


// ── RS485 FreeRTOS task (Core 1) ─────────────────────────────────────────────
// Runs handleRS485() in a tight loop on Core 1 with a short yield between
// iterations. This completely decouples RS485 response latency from HTTP/CAN
// work on Core 0. The MMDC gets a response within ~1ms of its poll regardless
// of what the web server or CAN handler is doing.
static void rs485Task(void *param) {
    for (;;) {
        handleRS485();
        vTaskDelay(1 / portTICK_PERIOD_MS);  // yield 1ms — same pattern as mefi CAN tasks.
                                              // Prevents WDT starvation while keeping
                                              // response latency well under MMDC timeout.
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

// ── /tune endpoint ───────────────────────────────────────────────────────────

// Helper: format a uint8_t as a 2-char hex string for HTML input defaults
String hexByte(uint8_t v) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", v);
    return String(buf);
}
// Helper: parse "0xNN" or "NN" hex string, return -1 on error
int parseHexArg(const String& s) {
    String t = s;
    t.trim();
    if (t.startsWith("0x") || t.startsWith("0X")) t = t.substring(2);
    if (t.length() == 0 || t.length() > 4) return -1;
    for (char c : t) if (!isxdigit(c)) return -1;
    return (int)strtol(t.c_str(), nullptr, 16);
}

void handleTuneGet() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Tune - NMEA2k RS485 Gateway</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0;padding:1em;box-sizing:border-box;}
    .card{background:#23272a;border-radius:12px;padding:2em 2.5em;box-shadow:0 2px 16px #000a;min-width:340px;max-width:520px;width:100%;}
    h2{color:#90caf9;margin-top:0;}h3{color:#90caf9;margin:1.2em 0 0.4em;font-size:1em;border-bottom:1px solid #333;padding-bottom:4px;}
    label{display:block;color:#b0c4d8;margin-top:0.9em;font-size:0.92em;}
    .hint{color:#888;font-size:0.80em;margin-top:2px;}
    input[type=number],input[type=text]{background:#181a1b;border:1px solid #444;border-radius:6px;color:#f1f1f1;padding:0.4em 0.7em;width:100%;box-sizing:border-box;font-size:1em;margin-top:4px;}
    .hexrow{display:grid;grid-template-columns:1fr 1fr 1fr;gap:0.7em;margin-top:0.5em;}
    .hexrow label{margin-top:0;}
    select{background:#181a1b;border:1px solid #444;border-radius:6px;color:#f1f1f1;padding:0.4em 0.7em;font-size:1em;width:100%;margin-top:4px;}
    input[type=submit]{background:#1976d2;color:#f1f1f1;padding:0.7em 2em;border:none;border-radius:6px;font-size:1em;cursor:pointer;margin-top:1.5em;width:100%;}
    input[type=submit]:hover{background:#1565c0;}
    .note{color:#ffcc80;font-size:0.83em;margin-top:1.2em;}
    .links{margin-top:1.4em;display:flex;gap:1.5em;flex-wrap:wrap;}
    .links a{color:#90caf9;}
    .banner{background:#1b3a1b;border:1px solid #388e3c;border-radius:8px;padding:0.7em 1em;color:#b0f2bc;font-size:0.88em;margin-bottom:1em;display:none;}
    .frame{background:#181a1b;border:1px solid #333;border-radius:6px;padding:0.5em 0.8em;font-family:monospace;font-size:0.85em;color:#aaa;margin-top:0.5em;word-break:break-all;}
  </style>
</head>
<body>
<div class='card'>
  <h2>🔧 Tuning Parameters</h2>
  <div class='banner' id='ok'>✓ Applied — no reboot needed.</div>
  <form method='POST' action='/tune'>

    <h3>Timing</h3>
    <label>Stale threshold (ms)
      <input type='number' name='staleMs' min='1000' max='60000' step='500' value=')rawliteral";
    html += String(tune.staleMs);
    html += R"rawliteral('></label>
    <div class='hint'>How long after last N2k frame before depth is stale. Default: 15000 ms.</div>

    <label>Pre-TX delay (μs)
      <input type='number' name='preTxUs' min='10' max='1000' step='10' value=')rawliteral";
    html += String(tune.preTxDelayUs);
    html += R"rawliteral('></label>
    <div class='hint'>DE assert to first byte gap. Default: 50 μs.</div>

    <h3>byte[11] — display mode</h3>
    <label>byte[11] FRESH (solid=0x02)
      <input type='text' name='freshByte11' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.freshByte11);
    html += R"rawliteral('></label>
    <div class='hint'>Sent when N2k depth is current. 0x02=solid confirmed. Try other values.</div>

    <label>byte[11] STALE
      <input type='text' name='staleByte11' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.staleByte11);
    html += R"rawliteral('></label>
    <div class='hint'>Sent when depth is stale. 0x02=solid, 0x00=blank. Looking for blink value.</div>

    <h3>Mystery bytes — reverse-engineer blink</h3>
    <div class='hint'>Response frame: [0D][09][b2][b3][d_lo][d_hi][b6][b7][b8][b9][tog][b11][CS]</div>
    <div class='hexrow'>
      <label>byte[2]
        <input type='text' name='b2' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b2);
    html += R"rawliteral('></label>
      <label>byte[3]
        <input type='text' name='b3' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b3);
    html += R"rawliteral('></label>
      <label>&nbsp;</label>
    </div>
    <div class='hint'>Default: 0x14 0xAA — meaning unknown.</div>
    <div class='hexrow'>
      <label>byte[6]
        <input type='text' name='b6' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b6);
    html += R"rawliteral('></label>
      <label>byte[7]
        <input type='text' name='b7' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b7);
    html += R"rawliteral('></label>
      <label>byte[8]
        <input type='text' name='b8' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b8);
    html += R"rawliteral('></label>
    </div>
    <div class='hexrow'>
      <label>byte[9]
        <input type='text' name='b9' maxlength='4' value='0x)rawliteral";
    html += hexByte(tune.b9);
    html += R"rawliteral('></label>
      <label>&nbsp;</label>
      <label>&nbsp;</label>
    </div>
    <div class='hint'>Default: 0xFF 0x03 0xFF 0x03 — possibly keel offset / secondary depth pair.</div>

    <h3>Depth override</h3>
    <label>Override depth value?
      <select name='overrideDepth'>
        <option value='0' )rawliteral";
    html += !tune.overrideDepth ? "selected" : "";
    html += R"rawliteral(>No — send real N2k depth</option>
        <option value='1' )rawliteral";
    html += tune.overrideDepth ? "selected" : "";
    html += R"rawliteral(>Yes — send override value below</option>
      </select>
    </label>
    <label>Override depth (raw tenths-of-foot, hex)
      <input type='text' name='depthOverride' maxlength='6' value='0x)rawliteral";
    html += String(tune.depthOverride, HEX);
    html += R"rawliteral('></label>
    <div class='hint'>e.g. 0x0059 = 8.9 ft, 0xFFFF = invalid marker. Active only when override is Yes.</div>

    <h3>Misc</h3>
    <label>Freeze toggle bit?
      <select name='freezeToggle'>
        <option value='0' )rawliteral";
    html += !tune.freezeToggle ? "selected" : "";
    html += R"rawliteral(>No — alternate normally (default)</option>
        <option value='1' )rawliteral";
    html += tune.freezeToggle ? "selected" : "";
    html += R"rawliteral(>Yes — freeze toggle (test: blink or blank?)</option>
      </select>
    </label>
    <div class='hint'>Frozen toggle previously caused blank. Testing whether it blinks the last known value.</div>

    <label>Respond to cmd=0x06 ping?
      <select name='pingEnabled'>
        <option value='0' )rawliteral";
    html += !tune.pingEnabled ? "selected" : "";
    html += R"rawliteral(>No (safe default)</option>
        <option value='1' )rawliteral";
    html += tune.pingEnabled ? "selected" : "";
    html += R"rawliteral(>Yes (causes RS485 collisions — test only)</option>
      </select>
    </label>

    <label>Boot fallback (no N2k data yet)
      <select name='bootFallback'>
        <option value='1' )rawliteral";
    html += tune.bootFallback ? "selected" : "";
    html += R"rawliteral(>Send 0.0 ft</option>
        <option value='0' )rawliteral";
    html += !tune.bootFallback ? "selected" : "";
    html += R"rawliteral(>Suppress (MMDC blank)</option>
      </select>
    </label>

    <input type='submit' value='Apply (no reboot)'>
  </form>
  <div class='note'>⚠ RAM-only — lost on reboot. OTA to make permanent.</div>
  <div class='links'><a href='/status'>Status</a> <a href='/drop'>Drop Test</a> <a href='/'>OTA</a></div>
</div>
<script>
  if (location.search.includes('ok')) {
    document.getElementById('ok').style.display = 'block';
    setTimeout(()=>document.getElementById('ok').style.display='none', 4000);
  }
</script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleTunePost() {
    if (server.hasArg("staleMs")) {
        int v = server.arg("staleMs").toInt();
        if (v >= 1000 && v <= 60000) tune.staleMs = (uint16_t)v;
    }
    if (server.hasArg("preTxUs")) {
        int v = server.arg("preTxUs").toInt();
        if (v >= 10 && v <= 1000) tune.preTxDelayUs = (uint16_t)v;
    }
    // Hex byte fields — accept 0xNN or NN
    auto applyHexByte = [&](const char* arg, uint8_t& field) {
        if (server.hasArg(arg)) {
            int v = parseHexArg(server.arg(arg));
            if (v >= 0 && v <= 0xFF) field = (uint8_t)v;
        }
    };
    applyHexByte("freshByte11",  tune.freshByte11);
    applyHexByte("staleByte11",  tune.staleByte11);
    applyHexByte("b2",           tune.b2);
    applyHexByte("b3",           tune.b3);
    applyHexByte("b6",           tune.b6);
    applyHexByte("b7",           tune.b7);
    applyHexByte("b8",           tune.b8);
    applyHexByte("b9",           tune.b9);
    if (server.hasArg("overrideDepth")) {
        tune.overrideDepth = server.arg("overrideDepth").toInt() != 0;
    }
    if (server.hasArg("depthOverride")) {
        int v = parseHexArg(server.arg("depthOverride"));
        if (v >= 0 && v <= 0xFFFF) tune.depthOverride = (uint16_t)v;
    }
    if (server.hasArg("freezeToggle")) tune.freezeToggle = server.arg("freezeToggle").toInt() != 0;
    if (server.hasArg("pingEnabled"))  tune.pingEnabled  = server.arg("pingEnabled").toInt()  != 0;
    if (server.hasArg("bootFallback")) tune.bootFallback = server.arg("bootFallback").toInt() != 0;

    Serial.printf("[TUNE] staleMs=%d preTxUs=%d freshB11=0x%02X staleB11=0x%02X "
                  "b2=0x%02X b3=0x%02X b6=0x%02X b7=0x%02X b8=0x%02X b9=0x%02X "
                  "overrideDepth=%d depthOverride=0x%04X freezeToggle=%d ping=%d boot=%d\n",
                  tune.staleMs, tune.preTxDelayUs,
                  tune.freshByte11, tune.staleByte11,
                  tune.b2, tune.b3, tune.b6, tune.b7, tune.b8, tune.b9,
                  tune.overrideDepth, tune.depthOverride,
                  tune.freezeToggle, tune.pingEnabled, tune.bootFallback);
    server.sendHeader("Location", "/tune?ok");
    server.send(303, "text/plain", "Redirecting...");
}

// ═══════════════════════════════════════════════════════════════════════════════
// /drop  —  Response-suppression endpoint for blink-timing experiments
//
// GET  /drop?ms=N   — suppress 0x09 responses for N ms (max 30000), then auto-resume
// GET  /drop?ms=0   — cancel an active drop window immediately
// GET  /drop        — show current drop status
//
// Safety:
//   • Self-cancelling: the window always expires, never needs manual reset
//   • Max window: 30 s (enough to observe blink + blank; won't leave display dark)
//   • Reads dropUntilMs; rs485Task sees the change atomically (single uint32 write)
// ═══════════════════════════════════════════════════════════════════════════════
void handleDrop() {
    uint32_t now = millis();
    String html;

    if (server.hasArg("ms")) {
        int ms = server.arg("ms").toInt();
        if (ms == 0) {
            // Cancel active drop
            dropUntilMs   = 0;
            statDropCount = 0;
            Serial.println("[DROP] Cancelled.");
            html = "<h2>Drop cancelled</h2><p>Responses resumed immediately.</p>";
        } else if (ms > 0 && ms <= 30000) {
            dropUntilMs   = now + (uint32_t)ms;
            statDropCount = 0;
            Serial.printf("[DROP] Suppressing responses for %d ms\n", ms);
            html  = "<h2>Drop active</h2>";
            html += "<p>Suppressing 0x09 responses for <b>" + String(ms) + " ms</b>.</p>";
            html += "<p>Auto-resumes at T+" + String(ms) + "ms. Watch the MMDC display.</p>";
            html += "<p>Record: when does it start blinking? When does it go blank?</p>";
            html += "<p><a href='/drop'>Refresh status</a> &nbsp; <a href='/drop?ms=0'>Cancel now</a></p>";
        } else {
            server.send(400, "text/plain", "ms must be 1-30000");
            return;
        }
    } else {
        // Status page
        bool active = (dropUntilMs > 0 && now < dropUntilMs);
        html  = "<h2>/drop — Response Suppression</h2>";
        if (active) {
            uint32_t remaining = dropUntilMs - now;
            html += "<p style='color:orange'><b>ACTIVE</b> — " + String(remaining) + " ms remaining, ";
            html += String(statDropCount) + " polls suppressed so far.</p>";
            html += "<p><a href='/drop?ms=0'>Cancel</a></p>";
        } else {
            html += "<p>Inactive. Polls suppressed in last window: " + String(statDropCount) + "</p>";
        }
        html += "<hr>";
        html += "<p>Suppress responses for: ";
        html += "<a href='/drop?ms=2000'>2s</a> &nbsp; ";
        html += "<a href='/drop?ms=5000'>5s</a> &nbsp; ";
        html += "<a href='/drop?ms=10000'>10s</a> &nbsp; ";
        html += "<a href='/drop?ms=20000'>20s</a> &nbsp; ";
        html += "<a href='/drop?ms=30000'>30s</a></p>";
        html += "<p><small>MMDC polls every ~2s — allow at least 2-3 missed polls to see display change.</small></p>";
    }

    String page = "<!DOCTYPE html><html><head><title>Drop</title>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<style>body{background:#181a1b;color:#f1f1f1;font-family:Arial,sans-serif;padding:2em;}";
    page += "a{color:#90caf9;} h2{color:#90caf9;}</style></head><body>";
    page += html;
    page += "<p><a href='/status'>Status</a> &nbsp; <a href='/tune'>Tune</a> &nbsp; <a href='/'>OTA</a></p>";
    page += "</body></html>";
    server.send(200, "text/html", page);
}

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
    <tr><td colspan='2' style='padding-top:12px;color:#90caf9;font-weight:bold'>Runtime Tuning</td></tr>
    <tr><td>Stale threshold</td><td id='tstale'>-</td></tr>
    <tr><td>Pre-TX delay</td><td id='tpretx'>-</td></tr>
    <tr><td>byte[11] fresh/stale</td><td id='tb11'>-</td></tr>
    <tr><td>Ping response</td><td id='tping'>-</td></tr>
    <tr><td>Boot fallback</td><td id='tboot'>-</td></tr>
    <tr><td>Raw RX total</td><td id='rawtotal'>-</td></tr>
    <tr>
      <td colspan='2'>
        <span style='color:#90caf9'>Raw RS485 RX (last 128 bytes)</span><br>
        <span class='hex' id='raw'>-</span>
      </td>
    </tr>
  </table>
  <br><a href='/tune'>Tune Parameters</a> &nbsp; <a href='/'>Firmware Update</a>
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
    document.getElementById('tstale').textContent = d.tune_stale_ms + ' ms';
    document.getElementById('tpretx').textContent = d.tune_pre_tx_us + ' μs';
    document.getElementById('tb11').textContent =
      '0x' + d.tune_fresh_b11.toString(16).padStart(2,'0') + ' / 0x' + d.tune_stale_b11.toString(16).padStart(2,'0');
    document.getElementById('tping').textContent = d.tune_ping_enabled ? 'yes' : 'no';
    document.getElementById('tboot').textContent = d.tune_boot_fallback ? 'send 0.0ft' : 'suppress';
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
    // Read shared state snapshot for HTTP response (Core 1 writes these)
    bool localEverHad_h;
    uint32_t localLastDepthMs_h;
    uint16_t localDepthTenths_h;
    if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localEverHad_h      = everHadDepth;
        localLastDepthMs_h  = lastDepthMs;
        localDepthTenths_h  = depthTenths;
        xSemaphoreGive(depthMutex);
    } else {
        localEverHad_h      = false;
        localLastDepthMs_h  = 0;
        localDepthTenths_h  = 0;
    }
    bool acquiredStale     = localEverHad_h && ((now - localLastDepthMs_h) > (uint32_t)tune.staleMs);
    bool fresh             = localEverHad_h && !acquiredStale;
    const char* depthState = fresh ? "fresh" : (acquiredStale ? "acquired_stale" : "boot_no_data");
    String json = "{";
    json += "\"firmware\":\""      + String(SW_VERSION_STRING) + "\",";
    json += "\"build_date\":\""    + String(SW_BUILD_DATE)     + "\",";
    json += "\"uptime_s\":"        + String(now / 1000)        + ",";
    json += "\"can_ready\":"       + String(canReady ? "true" : "false") + ",";
    json += "\"can_retries\":"     + String(statCanRetries)    + ",";
    json += "\"depth_ft\":"        + String(localDepthTenths_h / 10.0f, 1) + ",";
    json += "\"depth_state\":\""   + String(depthState)        + "\",";
    json += "\"depth_valid\":"     + String(fresh ? "true" : "false") + ",";
    json += "\"ever_had_depth\":"  + String(localEverHad_h ? "true" : "false") + ",";
    json += "\"depth_rx\":"        + String(statDepthRx)       + ",";
    json += "\"depth_age_ms\":"    + String(lastDepthRxMs ? (int32_t)(now - (uint32_t)lastDepthRxMs) : -1) + ",";
    json += "\"rs485_req\":"          + String(statRS485Req)       + ",";
    json += "\"rs485_ping_resp\":"     + String(statRS485PingResp)  + ",";
    json += "\"rs485_req_age_ms\":"    + String(lastRS485ReqMs  ? (int32_t)(now - lastRS485ReqMs)  : -1) + ",";
    json += "\"rs485_ping_age_ms\":"   + String(lastRS485PingMs ? (int32_t)(now - lastRS485PingMs) : -1) + ",";
    json += "\"rs485_tx_age_ms\":"     + String(lastRS485TxMs   ? (int32_t)(now - lastRS485TxMs)   : -1) + ",";
    json += "\"rs485_crc_fail\":"      + String(statRS485CrcFail)   + ",";
    json += "\"rs485_unknown\":"       + String(statRS485Unknown)    + ",";
    json += "\"rs485_last_crc_fail_hex\":\"" + lastCrcFailHex() + "\",";
    json += "\"rs485_stale_resp\":"    + String(statRS485StaleResp) + ",";
    // Expose current tuning params in /data for status page display
    json += "\"tune_stale_ms\":"        + String(tune.staleMs)        + ",";
    json += "\"tune_pre_tx_us\":"       + String(tune.preTxDelayUs)   + ",";
    json += "\"tune_fresh_b11\":"       + String(tune.freshByte11)    + ",";
    json += "\"tune_stale_b11\":"       + String(tune.staleByte11)    + ",";
    json += "\"tune_ping_enabled\":"    + String(tune.pingEnabled ? "true" : "false") + ",";
    json += "\"tune_boot_fallback\":"   + String(tune.bootFallback ? "true" : "false") + ",";
    json += "\"poll_interval_cnt\":"   + String(pollIntervalCnt)    + ",";
    json += "\"poll_interval_min_ms\":" + String(pollIntervalCnt ? pollIntervalMin : 0) + ",";
    json += "\"poll_interval_max_ms\":" + String(pollIntervalMax)   + ",";
    json += "\"poll_interval_avg_ms\":" + String(pollIntervalCnt ? (float)pollIntervalSum/pollIntervalCnt : 0.0f, 1) + ",";
    json += "\"raw_rx_total\":"  + String(rawRxTotal)        + ",";
    json += "\"raw_rx_hex\":\""  + rawRxHex()                + "\",";
    json += "\"drop_active\":"     + String((dropUntilMs > 0 && now < dropUntilMs) ? "true" : "false") + ",";
    json += "\"drop_remaining_ms\":" + String((dropUntilMs > 0 && now < dropUntilMs) ? (int32_t)(dropUntilMs - now) : 0) + ",";
    json += "\"drop_suppressed\":"  + String(statDropCount)    + "";
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
    server.on("/tune",   HTTP_GET,  handleTuneGet);
    server.on("/tune",   HTTP_POST, handleTunePost);
    server.on("/drop",   HTTP_GET,  handleDrop);
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
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED_KBPS))) {
        Serial.printf("WARNING: CAN init failed — will retry every %ds\n",
                      CAN_RETRY_MS / 1000);
        canReady = false;
    } else {
        Serial.printf("NMEA2000 CAN ready %dkbps\n", CAN_SPEED_KBPS);
        canReady = true;
    }

    // Create mutex for shared depth state between cores
    depthMutex = xSemaphoreCreateMutex();
    if (!depthMutex) {
        Serial.println("ERROR: failed to create depthMutex — halting");
        while (1) delay(1000);
    }

    // Spawn RS485 task on Core 1 at priority 2.
    // Arduino loop() runs on Core 1 at priority 1, so rs485Task preempts it
    // whenever UART data arrives — HTTP serving cannot delay RS485 responses.
    // RS485Serial is initialised above in setup() and accessed ONLY by rs485Task
    // after this point; no concurrent UART access from loop().
    xTaskCreatePinnedToCore(
        rs485Task,      // task function
        "rs485Task",    // name
        4096,           // stack bytes (ample for frame parser + sender)
        nullptr,        // param
        20,             // priority 20 — matches mefi CAN reader tasks;
                        // well above loop() (priority 1) and HTTP server
        nullptr,        // task handle (not needed)
        1               // core 1 (same as loop; preempts via priority)
    );
    Serial.println("RS485 task started on Core 1 at priority 20");
    Serial.println("Setup complete.");
}


// ═════════════════════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
    // RS485 is now handled by rs485Task on Core 1 — do NOT call handleRS485() here.
    // loop() handles CAN, HTTP, and CAN retry only.
    maybeRetryCAN();
    drainCAN();
    server.handleClient();
}
