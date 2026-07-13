# pi-test — Bench Test Harness

Lets you run the gateway off the water using a Raspberry Pi as a synthetic
NMEA 2000 depth source. Once the Pi is injecting depth, the full RS485 ↔ MMDC
path works on the bench exactly as it does on the boat.

## Hardware

```
Pi (MCP2515 + TJA1050 hat)
        │
   CAN bus (250 kbps, 2-wire + GND)
   [120Ω terminator at each end]
        │
ESP32 gateway (MCP2515 + TJA1050)
        │
   RS485 (76800 baud)
        │
Real MMDC gauge cluster
```

Two nodes on the CAN bus → you need **120Ω terminators at both ends**
(between CAN-H and CAN-L at the Pi hat and at the ESP32 board).

## Pi Setup

### 1. Enable MCP2515 in `/boot/config.txt`

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
dtoverlay=spi-bcm2835
```

- **oscillator**: check the crystal on your hat. Cheap modules are usually 8 MHz;
  PiCAN2 is 16 MHz (`oscillator=16000000`).
- **interrupt**: most hats use GPIO25. Check your hat's docs if CAN init fails.
- Reboot after editing.

### 2. Bring up the interface

```bash
chmod +x setup_can.sh
./setup_can.sh          # brings up can0 at 250 kbps
```

Or manually:
```bash
sudo ip link set can0 up type can bitrate 250000
```

Verify:
```bash
ip -details link show can0
candump can0             # should show frames when ESP32 is running
```

### 3. Install Python deps

No external deps — uses only stdlib (`socket`, `struct`, `json`, `urllib`).
Python 3.10+ required (uses `dict | None` type hint syntax).

## Running the Bench Test

### Step 1 — Inject depth

```bash
python3 send_depth.py --depth 8.9 --iface can0
```

- Default: 8.9 ft, 1 Hz, continuous.
- The gateway should receive PGN 128267 frames and the MMDC should show depth solid.
- Verify at http://192.168.4.1/status

Options:
```
--depth 8.9     Depth in feet
--iface can0    SocketCAN interface
--interval 1.0  Send interval (seconds)
--count 0       Number of frames (0 = infinite)
```

### Step 2 — Run the drop test (in a second terminal)

```bash
python3 drop_test.py --ms 10000
```

This:
1. Verifies the gateway is reachable on its WiFi AP (`nmea2k_rs485_gw`)
2. Prompts you to get eyes on the MMDC display
3. Triggers `/drop?ms=10000` — suppresses 0x09 responses for 10 s
4. Polls `/data` every 250 ms and logs which polls were suppressed
5. Auto-resumes, watches recovery for 5 more seconds

**While the test runs, watch the MMDC display and note:**
- When does it start **blinking**?
- When does it go **blank** / show "No Response"?
- When does **solid depth return** after the window ends?

Compare those timestamps to the logged poll suppression timeline.

Options:
```
--host 192.168.4.1   ESP32 gateway IP (default: gateway WiFi AP)
--ms   10000         Drop window ms (max 30000)
--poll 0.25          Polling interval seconds
```

### Manual drop via browser

Connect to `nmea2k_rs485_gw` WiFi, open http://192.168.4.1/drop

Quick links: 2s / 5s / 10s / 20s / 30s windows. Self-cancelling — safe to use
while stationary. Cancel early with `/drop?ms=0`.

## What We're Testing

The MMDC blinks when the factory Lowrance LSM-3 depth sonar loses bottom.
We know:
- `byte[11] = 0x02` → solid display ✓
- `byte[11] = 0x00` → blank display (confirmed accidental)
- **No blink flag found in the response frame**

Working theory: blink is MMDC-internal behaviour triggered by *response
absence* (timeout). The LSM-3 simply stopped responding when it lost bottom.

This test answers:
1. How many missed polls (at ~2s each) before the display blinks?
2. How long after blink until it goes blank?
3. Does it auto-recover when responses resume, or need a power cycle?

## Files

| File             | Purpose                                      |
|------------------|----------------------------------------------|
| `setup_can.sh`   | Bring up MCP2515 SocketCAN interface         |
| `send_depth.py`  | Continuously inject PGN 128267 onto CAN bus  |
| `drop_test.py`   | Trigger /drop and log suppression timeline   |
