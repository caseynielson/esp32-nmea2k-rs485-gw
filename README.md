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

### The fix (v2.15+)

RS485 polling runs in a dedicated FreeRTOS task at **priority 20**, pinned to Core 1. This completely preempts `loop()` (priority 1) the instant UART bytes arrive.

```
Core 1:
  rs485Task  (priority 20)  ← preempts loop() instantly on UART data
  loop()     (priority  1)  ← CAN reads, HTTP, OTA
```

`depthMutex` protects the shared depth state between cores. After extended on-water testing this architecture has proven reliable — zero "No Response" events observed.

## RS485 Protocol (Medallion MMDC) — Fully Reverse-Engineered

### Depth poll request (MMDC → gateway, 4 bytes)

| Byte | Value  | Description                          |
|------|--------|--------------------------------------|
| 0    | `0x04` | Frame length                         |
| 1    | `0x09` | Command: depth poll                  |
| 2    | —      | Payload byte (varies)                |
| 3    | —      | Two's-complement checksum            |

### Depth response (gateway → MMDC, 13 bytes)

| Byte | Value        | Description                                        |
|------|--------------|----------------------------------------------------|
| 0    | `0x0D` (13)  | Frame length                                       |
| 1    | `0x09`       | Command echo                                       |
| 2    | `0x14`       | **Required** — any other value → "No Response"     |
| 3    | `0xAA`       | **Required** — any other value → "bad status"      |
| 4    | depth_lo     | Depth low byte (tenths of a foot, little-endian)   |
| 5    | depth_hi     | Depth high byte (tenths of a foot, little-endian)  |
| 6    | `0xFF`       | Filler — no effect when changed                    |
| 7    | `0x03`       | Filler — no effect when changed                    |
| 8    | `0xFF`       | Filler — no effect when changed                    |
| 9    | `0x03`       | Filler — no effect when changed                    |
| 10   | toggle       | Alternates 0/1 each response — **must alternate**  |
| 11   | `0x02`       | `0x02` = solid display, `0x00` = blank display     |
| 12   | checksum     | Two's-complement checksum over bytes 0–11          |

**Depth encoding:** value = `round(depth_ft × 10)`, little-endian uint16.
Example: 12.3 ft → `0x7B 0x00` (123 decimal).

**Checksum:** `checksum = (~sum(bytes[0..11]) + 1) & 0xFF`

### Display state machine

| State | Depth value sent | Toggle | MMDC display |
|-------|-----------------|--------|--------------|
| Fresh N2K depth | Real tenths-of-foot | Alternates | Solid depth value |
| Stale (no N2K > 5s) | `0xFFFF` | Alternates | Blinks "400" (lost-bottom) |
| No response | — | — | "No Response" after timeout |

**Key findings from reverse-engineering (2026-07-20):**
- `byte[2] = 0x14` and `byte[3] = 0xAA` are required — non-negotiable
- `byte[11]` controls solid (`0x02`) vs blank (`0x00`) — NOT blink
- **Toggle bit must always alternate** — frozen toggle causes "No Response" blank, not blink
- **`0xFFFF` depth value** is the NMEA 2000 "not available" sentinel — MMDC recognises it and blinks "400 ft" (its maximum range / lost-bottom indicator)
- Blinking the last known depth value (original LSM-3 behavior) cannot be replicated — the MMDC does not expose this via the RS485 protocol. "400 blinking" is the closest achievable equivalent.

### MMDC bus cycle (observed, ~8.7 ms period)

Each MMDC cycle:
1. Status broadcasts: `04 03 05 F4` + `08 03 81 02 00 00 00 72` × 4–5 times
2. Ping frame: `04 06 XX CS` — **do not respond** (no response window; responding causes RS485 collision)
3. Depth poll: `04 09 XX CS` — respond within ~1 ms with 13-byte depth frame (~every 2 s)

## NMEA 2000 PGN 128267 — Water Depth

| Field  | Bytes | Scale     | Notes                  |
|--------|-------|-----------|------------------------|
| SID    | 0     | —         | Sequence ID (ignored)  |
| Depth  | 1–3   | 0.01 m    | 24-bit unsigned        |
| Offset | 4–5   | 0.001 m   | 16-bit signed          |
| Range  | 6     | 10 m      | 0xFF = unknown         |

**CAN ID for PGN 128267:** `0x19F50B01` (priority 6, SA=0x01)
- PGN 128267 = `0x01F50B`: DP=1, PF=0xF5, PS=0x0B (PDU2 broadcast)
- 29-bit ID: `(6<<26) | (1<<24) | (0xF5<<16) | (0x0B<<8) | SA`

**Frame format:** The gateway reads `data[1..4]` directly as the 32-bit depth value — **no fast-packet reassembly**. Send raw PGN payload bytes without a fast-packet header.

## Web Interface

Connect to WiFi AP **`nmea2k_rs485_gw`** (password `123456789`) then:

| URL                           | Description                              |
|-------------------------------|------------------------------------------|
| `http://192.168.4.1/`         | Redirect to /status                      |
| `http://192.168.4.1/status`   | Live status dashboard                    |
| `http://192.168.4.1/data`     | JSON data endpoint                       |
| `http://192.168.4.1/tune`     | Runtime parameter tuning (no reflash)    |
| `http://192.168.4.1/drop`     | Suppress RS485 responses for N ms (test) |
| `http://192.168.4.1/ota`      | OTA firmware update                      |

### `/tune` parameters

| Parameter      | Default  | Notes |
|----------------|----------|-------|
| `staleMs`      | 5000     | ms after last N2K frame before depth considered stale |
| `preTxDelayUs` | 50       | DE assert delay before RS485 TX (µs) |
| `freshByte11`  | `0x02`   | byte[11] when fresh — keep `0x02` (solid) |
| `staleByte11`  | `0x02`   | byte[11] when stale — no effect on blink, keep `0x02` |
| `b2`–`b9`      | see above| Mystery bytes — tunable for experiments |
| `overrideDepth`| false    | Send fixed depth value instead of real N2K depth |
| `depthOverride`| `0xFFFF` | Raw tenths-of-foot override value |
| `freezeToggle` | false    | Freeze toggle bit — confirmed causes blank, not blink |
| `pingEnabled`  | false    | Respond to 0x06 ping — **leave false** (RS485 collision) |
| `bootFallback` | true     | Send 0.0 ft before first N2K depth received |

## Pi Test Harness

A Raspberry Pi with an MCP2515 CAN hat can inject synthetic NMEA 2000 depth frames for bench testing without being on the water.

### Setup

```bash
# Bring up SocketCAN interface (250 kbps)
./pi-test/setup_can.sh

# Inject fixed depth
python3 pi-test/send_depth.py --depth 8.9 --iface can0

# Walk depth 5.0→15.0 ft in 0.1 ft steps (bouncing) — confirms display is live
python3 pi-test/send_depth.py --start 5.0 --end 15.0 --iface can0
```

### CAN ID note

PGN 128267 CAN ID = `0x19F50B01`. The EFF flag (`0x80000000`) must be OR'd in for SocketCAN:
```python
CAN_EFF_ID = 0x19F50B01 | 0x80000000  # = 0x99F50B01
```

### Frame format note

The gateway reads depth from `data[1..4]` directly. Send **raw PGN payload** (no fast-packet header):
```
data[0] = SID
data[1] = depth byte 0 (LSB)
data[2] = depth byte 1
data[3] = depth byte 2
data[4] = 0x00
data[5] = offset LSB
data[6] = offset MSB
data[7] = range (0xFF)
```
A fast-packet header (`seq|0x00`, `total_len`) in bytes 0–1 puts `0x07` (length) into `data[1]`, which produces rawM > 30000 and the frame is dropped.

## Dependencies

- [ESP32-TWAI-CAN](https://github.com/handmade0octopus/ESP32-TWAI-CAN)
- Arduino ESP32 core 2.0.17 (**do not upgrade** — core 3.x breaks this sketch)
- Standard Arduino libraries: `WiFi`, `WebServer`, `Update`

## Building

Open `esp32_nmea2k_rs485_v2.ino` in Arduino IDE. Board: **ESP32 Dev Module**, core **2.0.17**. Flash via USB or OTA (`/ota`).

> **Note:** ArduinoOTA (UDP/mDNS) is disabled — it starts a background mDNS task that interferes with USB-CDC serial and causes Arduino IDE lockup. Use the HTTP `/ota` page instead.

## Changelog

### v2.19.2 — 2026-07-20 ✅ Current
- **Feature:** Add `freezeToggle` to `/tune` — confirmed frozen toggle causes blank, not blink
- **Tune:** All mystery frame bytes (`b2`, `b3`, `b6`–`b9`) now individually tunable from `/tune`
- **Tune:** Free hex input for `byte[11]` (was limited to 0x00/0x02 dropdown)
- **Tune:** Depth override with arbitrary raw value (e.g. `0xFFFF`)

### v2.19.1 — 2026-07-20
- **Tune:** Default `staleMs` reduced from 15 s → 5 s

### v2.19.0 — 2026-07-20
- **Feature:** Send `0xFFFF` when N2K depth stale — MMDC blinks "400" (lost-bottom indicator)
- Confirmed by experiment: `0xFFFF` is the NMEA 2000 "not available" sentinel; MMDC recognises it natively and blinks its maximum range value
- Accepted as final stale behavior — blinking last known value not achievable via this protocol

### v2.18.0 — 2026-07-20
- **Feature:** Expand `/tune` to expose all mystery frame bytes for runtime experimentation
- Added depth override, free hex byte[11] input, b2/b3/b6–b9 fields
- Reverse-engineering findings documented: byte[2]=0x14 and byte[3]=0xAA required; bytes[6–9] have no observed effect; byte[11] controls solid/blank not blink

### v2.16.0–v2.17.0 — 2026-07-20
- Bench test infrastructure: Raspberry Pi CAN injection confirmed working
- Fixed `send_depth.py`: wrong CAN ID (0x18F5FF01 → 0x19F50B01) and fast-packet header bug
- v2.17.0 reverted (incorrect stale-suppression approach)

### v2.15.1 — 2026-07-02
- **Reliability:** RS485 task priority raised to 20. Zero "No Response" events in field testing.

### v2.15.0 — 2026-07-02
- **Architecture:** RS485 polling moved to dedicated FreeRTOS task (Core 1, priority 20)
- `depthMutex` added for cross-core depth state protection

### v2.14.0 — 2026-07-02
- `pingEnabled` defaulted back to `false` (v2.13 regression — responding to 0x06 ping causes RS485 collision)

### v2.13.0 — 2026-07-02
- `/tune` HTTP endpoint added

### v2.12.0 — 2026-07-02
- Respond to cmd=0x06 ping + poll interval tracking (later found to cause collision — disabled by default)

### v2.7.0–v2.11.0 — 2026-06-28 to 2026-07-01
- Stale threshold 5 s → 15 s (later reduced back to 5 s in v2.19.1)
- CAN drain 1 → 8 frames per iteration
- RS485 RX ring buffer 128 bytes
- `FRAME_MAX_LEN` 16 → 32
- Pre-TX delay 200 µs → 50 µs
- Three-state depth model: fresh / acquired-stale / boot-no-data
- Toggle confirmed: frozen = blank (not blink)

### v2.4.0 — 2026-06-28
- DE/RE hold increased 100 µs → 250 µs (was dropping stop bit)
- CAN auto-retry on boot failure
- Raw RS485 RX sniffer on `/status`

### v2.0.0–v2.3.x — 2026-04-16
- FreeRTOS attempt (caused USB-CDC lockup), reverted, rearchitected

### v1.5.1 — 2025-09-11
- Initial working version
