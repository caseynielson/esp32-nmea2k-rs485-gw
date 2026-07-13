# esp32-nmea2k-rs485-gw

ESP32 gateway that bridges **NMEA 2000 water depth** (PGN 128267) to the **Medallion MMDC** proprietary RS485 protocol.

## Overview

The Medallion MMDC instrument cluster polls for water depth over a proprietary RS485 bus at 76800 baud. This gateway listens on NMEA 2000 for PGN 128267, converts the depth to tenths-of-feet, and answers the MMDC's polling requests in real time.

```
NMEA 2000 network                ESP32 gateway               Medallion MMDC
──────────────────    CAN (250k)  ──────────────  RS485 (76800)  ──────────────
 Depth sonar/GPS  ──────────────▶ TJA1050 → ESP ──────────────▶  Display
  PGN 128267                         MAX485
```

## Hardware

| Signal       | ESP32 GPIO | Transceiver pin |
|--------------|-----------|-----------------|
| CAN TX       | 5         | TJA1050 TXD     |
| CAN RX       | 4         | TJA1050 RXD     |
| RS485 TX     | 17        | MAX485 DI       |
| RS485 RX     | 16        | MAX485 RO       |
| RS485 DE/RE  | 21        | MAX485 DE + RE  |

- CAN speed: **250 kbps** (NMEA 2000 standard)
- RS485 baud: **76800** (Medallion MMDC proprietary)

## Software Architecture

### The reliability problem (v1.x / v2.1–v2.14)

The original single-task loop ran the WebServer, CAN reads, and RS485 reads all in sequence. The WebServer library (`server.handleClient()`) introduces synchronous latency spikes of 10–100 ms. If a spike landed between the MMDC sending its depth poll and the ESP32 responding, the MMDC timed out and showed "No Response".

Early attempts at FreeRTOS core-pinning (v2.0) caused USB-CDC serial monitor lockup in the Arduino IDE by starving the Arduino `loopTask`. Time-slicing the web server in `loop()` (v2.1–v2.14) improved things but didn't fully eliminate the problem.

### The fix (v2.15+)

RS485 polling runs in a dedicated FreeRTOS task at **priority 20**, pinned to Core 1. This completely preempts `loop()` (priority 1) the instant UART bytes arrive.

```
Core 1:
  rs485Task  (priority 20)  ← preempts loop() instantly on UART data
  loop()     (priority  1)  ← CAN reads, HTTP, OTA
```

`depthMutex` protects the shared depth state between cores. A `vTaskDelay(1 / portTICK_PERIOD_MS)` yield inside the task loop prevents starvation of lower-priority tasks.

After a week of on-water testing this architecture has proven reliable — zero "No Response" events observed.

## RS485 Protocol (Medallion MMDC)

### Request (MMDC → gateway, 4 bytes)

| Byte | Value  | Description        |
|------|--------|--------------------|
| 0    | `0x04` | Message length     |
| 1    | `0x09` | Command: get depth |
| 2-3  | —      | Two's-complement checksum over bytes 0-1 (see below) |

### Response (gateway → MMDC, 13 bytes)

| Byte | Value        | Description                        |
|------|--------------|------------------------------------|
| 0    | `0x0D` (13)  | Message length                     |
| 1    | `0x09`       | Command echo                       |
| 2    | `0x14`       | Sub-command                        |
| 3    | `0xAA`       | Status/ID                          |
| 4    | depth_lo     | Depth low byte (tenths of a foot)  |
| 5    | depth_hi     | Depth high byte (tenths of a foot) |
| 6-7  | `0xFF 0x03`  | Filler                             |
| 8-9  | `0xFF 0x03`  | Filler                             |
| 10   | toggle       | Bit toggled each response          |
| 11   | `0x02`       | Filler                             |
| 12   | checksum     | Two's-complement checksum          |

**Depth encoding:** value = depth_in_feet × 10, rounded to nearest integer.  
Example: 12.3 ft → `0x7B 0x00` (123 decimal).

**Checksum:** 8-bit two's complement of all preceding bytes.  
`checksum = (~sum(bytes[0..n-2]) + 1) & 0xFF`

**Depth state machine (three states):**
- **Fresh** — depth received within `staleMs` (default 15 s). Displayed solid.
- **Acquired-stale** — depth not updated recently, but we've seen depth before. Holds last known value, displayed solid. Garmin MFD already blinks its depth field when invalid, so holding the last value is useful context.
- **Boot/no-data** — no depth ever received. Responds with 0.0 ft.

**Known limitation:** The original factory display blinked when the transducer lost bottom. We have not yet identified a mechanism to trigger MMDC display blinking from the response frame — byte[11] controls solid vs. blank (not blink). The blink is believed to be MMDC-internal behaviour triggered by response absence, not a flag in the response frame. Investigation deferred.

## NMEA 2000 PGN 128267 — Water Depth

| Field  | Bytes | Scale     | Notes                  |
|--------|-------|-----------|------------------------|
| SID    | 0     | —         | Sequence ID (ignored)  |
| Depth  | 1-3   | 0.01 m    | 24-bit unsigned        |
| Offset | 4-5   | 0.001 m   | 16-bit signed          |
| Range  | 6     | 10 m      | 0xFF = unknown         |

Conversion: `depth_ft = depth_m × 3.28084`, then `depth_tenths = round(depth_ft × 10)`.

## Web Interface

Connect to WiFi AP **`nmea2k_rs485_gw`** (password `123456789`) then:

| URL            | Description                        |
|----------------|------------------------------------|
| `http://192.168.4.1/`          | Firmware update (HTTP file upload)  |
| `http://192.168.4.1/logs.html` | Live log viewer with debug toggles  |
| `http://192.168.4.1/status`    | JSON status (depth, stats, uptime)  |
| `http://192.168.4.1/tune`      | Runtime parameter adjustment (no reflash needed) |

> **Note:** ArduinoOTA (UDP/mDNS) is intentionally disabled — `ArduinoOTA.begin()` starts a background mDNS task that interferes with USB-CDC serial and causes Arduino IDE lockup on upload. Use the HTTP `/update` page for firmware updates instead.

### `/tune` parameters

| Parameter     | Default  | Notes |
|---------------|----------|-------|
| `staleMs`     | 15000    | How long before depth is considered stale (ms) |
| `preTxDelayUs`| 50       | DE assert delay before RS485 TX (µs) |
| `freshByte11` | `0x02`   | Byte[11] when depth is fresh — **do not set to 0x00** (blanks display) |
| `staleByte11` | `0x02`   | Byte[11] when depth is stale |
| `pingEnabled` | `false`  | Respond to 0x06 ping frames — **leave false** (causes RS485 collision) |
| `bootFallback`| `true`   | Respond 0.0 ft before first N2K depth received |

### `/status` JSON example

```json
{
  "version": "v2.0.0",
  "uptime_s": 3742,
  "depth_ft": 12.3,
  "depth_valid": true,
  "depth_rx": 1250,
  "rs485_req": 3741,
  "rs485_bad": 0,
  "dbg_nmea": false,
  "dbg_rs485": false
}
```

## Serial Console (115200 baud)

Serial commands are minimal — full debug control is available via the web interface at `/logs.html`.

| Command     | Description                   |
|-------------|-------------------------------|
| `nmea on`   | Enable NMEA 2000 debug output |
| `nmea off`  | Disable NMEA 2000 debug output|
| `rs485 on`  | Enable RS485 debug output     |
| `rs485 off` | Disable RS485 debug output    |

## Test Harness

The full test loop:

```
Raspberry Pi                      ESP32 gateway              MMDC mimic (esp32_mimic_mmdc)
────────────                      ─────────────              ─────────────────────────────
canplayer replays .log  ────────▶ receives PGN 128267  ◀──── polls depth every 1 s
(NMEA 2000 CAN frames)            converts & caches          prints depth to Serial
```

- **Pi side:** `canplayer` or `cangen` on the SocketCAN interface, replaying a capture that includes PGN 128267 frames.
- **MMDC mimic:** [`esp32_mimic_mmdc`](https://github.com/caseynielson/esp32_mimic_mmdc) — same MAX485 pinout, sends `0x04 0x09 0x11 0xE2` every second and prints the extracted depth. Confirms the gateway is responding correctly without needing a real MMDC on the bench.

## Dependencies

- [ESP32-TWAI-CAN](https://github.com/handmade0octopus/ESP32-TWAI-CAN)
- Arduino ESP32 core ≥ 2.x
- Standard Arduino libraries: `WiFi`, `WebServer`, `Update`

## Building

Open `esp32_nmea2k_rs485_v2.ino` in Arduino IDE. Select board **ESP32 Dev Module** (or your specific variant). Flash normally or use the OTA update page after first flash.

## Changelog

### v2.15.1 — 2026-07-02 ✅ Current (field-tested)
- **Reliability:** RS485 task priority raised from 2 → 20 to match the mefi-nmea2k-gateway pattern. Ensures rs485Task preempts any priority-1 work without contest.
- **Status:** One week of on-water testing confirmed zero "No Response" events. Depth solid on both gauge cluster and MMDC MFD depth screen.

### v2.15.0 — 2026-07-02
- **Architecture change:** RS485 polling moved from `loop()` to a dedicated FreeRTOS task (Core 1, priority 2). Completely eliminates missed poll responses caused by `server.handleClient()` blocking `loop()` for 10–100 ms.
- Added `depthMutex` to protect shared depth state between cores.

### v2.14.0 — 2026-07-02
- **Regression fix:** `pingEnabled` defaulted to `true` in v2.13.0 — reverted to `false`. Responding to 0x06 ping causes RS485 collision because MMDC issues 0x06 and 0x09 back-to-back with zero gap; our 13-byte response overlaps the MMDC's next broadcast.
- `/tune` endpoint retained for diagnostic use.

### v2.13.0 — 2026-07-02
- **Feature:** `/tune` HTTP endpoint — adjust runtime parameters without reflash (`staleMs`, `preTxDelayUs`, `freshByte11`, `staleByte11`, `pingEnabled`, `bootFallback`).
- Blink hypothesis support added (byte[11] tunable). Later confirmed: no blink flag exists in the depth response frame.

### v2.12.0 — 2026-07-02
- **Feature:** Respond to cmd=0x06 ping frames + poll interval tracking (min/avg/max) on `/status`.
- **Regression:** 0x06 response caused RS485 collision — reverted in v2.14.0.

### v2.7.0–v2.11.0 — 2026-06-28 to 2026-07-01
- Depth stale threshold increased 5 s → 15 s
- CAN drain increased 1 → 8 frames per `loop()` iteration
- RS485 RX ring buffer increased to 128 bytes
- `FRAME_MAX_LEN` increased 16 → 32 to handle longer unknown MMDC frames
- Pre-TX delay reduced 200 µs → 50 µs
- Three-state depth model: fresh / acquired-stale / boot-no-data
- Toggle bit always alternates (frozen toggle causes MMDC blank, not blink — confirmed)
- `statRS485StaleResp` counter added

### v2.4.0 — 2026-06-28
- **Bugfix:** Post-transmit DE/RE hold increased from 100 µs → 250 µs. At 76800 baud one byte takes ~130 µs to clock out of the UART shift register after `flush()` returns. The previous 100 µs could drop the driver enable before the stop bit finished, corrupting the last byte of every response.
- **Feature:** CAN auto-retry — if `ESP32Can.begin()` fails at boot, `maybeRetryCAN()` reattempts every 5 s. NMEA 2000 networks can take a few seconds to stabilise after power-on, so the initial attempt can fail even when hardware is fine. Retry count visible on status page.
- **Feature:** Raw RS485 RX sniffer — every byte received from the MMDC (before framing/checksum) is captured in a 64-byte ring buffer and displayed on the `/status` page as hex. Shows unknown message types that the current framer discards, making it possible to spot protocol issues without a logic analyser.
- **Feature:** `/status` page now shows CAN ready state (green/red), CAN retry count, RS485 bad-frame count highlighted in amber, raw RX byte count, and the live raw hex dump.
- **Feature:** `/data` JSON adds `can_ready`, `can_retries`, `raw_rx_total`, `raw_rx_hex` fields.

### v2.1.0 — 2026-04-16
- **Bugfix:** Removed `xTaskCreatePinnedToCore` entirely — pinning rs485Task to Core 1 at priority 10 was starving the Arduino `loopTask` and causing USB-CDC serial monitor lockup / IDE freeze on upload
- RS485 handling moved back to `loop()` as first call, time-sliced web/OTA handlers prevent them from blocking
- No mutexes needed (single-task, single Serial owner)
- `handleWeb()` and `handleOTA()` rate-limited with millis() guards (5ms and 10ms budgets)

### v2.0.1 — 2026-04-16
- **Bugfix:** `logMsg()` no longer calls `Serial.write()` directly — was causing USB-CDC serial monitor lockup and Arduino IDE freeze when called from Core 1
- Core 0 now owns all Serial output via `drainLogToSerial()` in `loop()`
- Bumped `rs485Task` stack from 2 kB → 4 kB for headroom

### v2.0.0 — 2026-04-16
- **Reliability fix:** RS485 response task pinned to Core 1 at priority 10 — completely decoupled from web server latency
- **CAN:** `drainCAN()` now empties the full TWAI RX queue per loop iteration instead of reading one frame
- **Mutex:** `depthMutex` protects shared depth state between cores
- **New:** `/status` JSON endpoint
- **Cleanup:** removed dead code, simplified PGN table, log writes are mutex-protected

### v2.1.0–v2.3.7 — 2026-04-16 to 2026-06-27
- Removed FreeRTOS core-pinning (was causing Arduino IDE USB-CDC lockup)
- RS485 moved back to `loop()` as first call; time-sliced web/OTA handlers
- Various label and diagnostic tweaks

### v2.0.0 — 2026-04-16
- First FreeRTOS attempt: RS485 task on Core 1 priority 10
- Caused USB-CDC serial lockup in Arduino IDE; abandoned

### v1.5.1 — 2025-09-11
- Initial working version (single-task, reliability issues under web server load)
