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

### The reliability problem (v1.x)

The original single-task loop ran `ArduinoOTA.handle()`, `server.handleClient()`, CAN reads, and RS485 reads all in sequence. The WebServer library introduces multi-millisecond latency spikes. If a spike happened between the MMDC sending its depth request and the ESP32 reading and responding, the display went blank.

### The fix (v2.x)

Two-core split:

```
Core 1 — rs485Task (priority 10, dedicated)
  Blocks on UART byte arrival.
  Assembles 4-byte request, validates checksum, fires 13-byte response.
  Turnaround time: < 1 ms.
  Never preempted by web server or OTA.

Core 0 — loop() (normal priority)
  Drains full TWAI CAN RX queue each iteration (not just one frame).
  Runs ArduinoOTA.handle(), server.handleClient(), serial console.
  Latency spikes here are harmless.
```

Shared state (`depthTenths`, `depthValid`, `lastDepthMs`) is protected by a FreeRTOS mutex so both cores can safely read and write.

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

If no valid NMEA 2000 depth has been received within 5 seconds, the gateway responds with depth = 0 (display shows no depth rather than stale data).

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
| `http://192.168.4.1/`          | OTA firmware update page |
| `http://192.168.4.1/logs.html` | Live log viewer with debug toggles |
| `http://192.168.4.1/status`    | JSON status (depth, stats, uptime) |

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

| Command           | Description                         |
|-------------------|-------------------------------------|
| `depth`           | Print current depth and staleness   |
| `stats`           | Print depth/RS485 counters, uptime  |
| `debug nmea on`   | Enable verbose NMEA 2000 log output |
| `debug nmea off`  | Disable NMEA 2000 log output        |
| `debug rs485 on`  | Enable verbose RS485 log output     |
| `debug rs485 off` | Disable RS485 log output            |
| `help`            | Show command list                   |

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
- Standard Arduino libraries: `WiFi`, `WebServer`, `ArduinoOTA`, `Update`

## Building

Open `esp32_nmea2k_rs485_v2.ino` in Arduino IDE. Select board **ESP32 Dev Module** (or your specific variant). Flash normally or use the OTA update page after first flash.

## Changelog

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

### v1.5.1 — 2025-09-11
- Initial working version (single-task, reliability issues under web server load)
