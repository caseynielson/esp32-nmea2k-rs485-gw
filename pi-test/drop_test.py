#!/usr/bin/env python3
"""
drop_test.py — Trigger a /drop window on the ESP32 gateway and monitor
               the /data JSON endpoint to log exactly when polls are suppressed.

Usage:
    python3 drop_test.py [--host 192.168.4.1] [--ms 10000]

What this tells you:
    - How many polls the MMDC sends before display changes
    - When (wall-clock + relative ms) each suppressed poll happened
    - When the window expired and responses resumed

You watch the MMDC display during the run and note:
    - T_blink: when did it start blinking?
    - T_blank: when did it go blank?
    - T_recover: when did solid depth return after window expired?

Then compare those observations to the logged poll timeline.
"""

import argparse
import json
import time
import urllib.request
import urllib.error
import sys


def fetch_data(host: str, timeout: float = 2.0) -> dict | None:
    url = f"http://{host}/data"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception as e:
        print(f"  [warn] /data fetch failed: {e}")
        return None


def trigger_drop(host: str, ms: int) -> bool:
    url = f"http://{host}/drop?ms={ms}"
    try:
        with urllib.request.urlopen(url, timeout=3.0) as r:
            r.read()
        return True
    except Exception as e:
        print(f"  [error] /drop trigger failed: {e}")
        return False


def cancel_drop(host: str) -> None:
    try:
        with urllib.request.urlopen(f"http://{host}/drop?ms=0", timeout=2.0) as r:
            r.read()
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description='Trigger /drop and log poll suppression timeline')
    parser.add_argument('--host', default='192.168.4.1', help='ESP32 gateway WiFi IP (default: 192.168.4.1)')
    parser.add_argument('--ms',   type=int, default=10000, help='Drop window duration in ms (default: 10000)')
    parser.add_argument('--poll', type=float, default=0.25, help='Polling interval in seconds (default: 0.25)')
    args = parser.parse_args()

    print(f"[drop_test] Target: http://{args.host}")
    print(f"[drop_test] Drop window: {args.ms} ms ({args.ms/1000:.1f}s)")
    print(f"[drop_test] Poll interval: {args.poll*1000:.0f} ms")
    print()

    # Pre-check: verify gateway is reachable and responding
    print("Checking gateway...", end=' ', flush=True)
    baseline = fetch_data(args.host)
    if not baseline:
        print("FAILED. Is the ESP32 WiFi AP up and you're connected to nmea2k_rs485_gw?")
        sys.exit(1)
    print(f"OK — firmware {baseline.get('firmware','?')}, depth={baseline.get('depth_ft','?')} ft")
    print()

    # Baseline stats
    pre_req  = baseline.get('rs485_req', 0)
    pre_drop = baseline.get('drop_suppressed', 0)

    print("="*60)
    print("INSTRUCTIONS:")
    print("  Watch the MMDC depth display closely.")
    print("  Note the time when:")
    print("    1. Display starts BLINKING")
    print("    2. Display goes BLANK / shows 'No Response'")
    print("    3. Display returns to SOLID after window ends")
    print("="*60)
    print()
    input("Press ENTER to start the drop test...")
    print()

    t_start = time.time()

    # Trigger the drop
    print(f"[{0:6.2f}s] Triggering /drop?ms={args.ms} ...", end=' ', flush=True)
    if not trigger_drop(args.host, args.ms):
        print("FAILED — aborting.")
        sys.exit(1)
    print("OK. Window active. Watching...")
    print()

    last_suppressed = pre_drop
    last_req        = pre_req

    try:
        while True:
            time.sleep(args.poll)
            elapsed = time.time() - t_start
            data = fetch_data(args.host)
            if not data:
                continue

            active     = data.get('drop_active', False)
            remaining  = data.get('drop_remaining_ms', 0)
            suppressed = data.get('drop_suppressed', 0)
            total_req  = data.get('rs485_req', 0)
            depth_ft   = data.get('depth_ft', '?')

            new_suppressed = suppressed - last_suppressed
            new_req        = total_req  - last_req
            last_suppressed = suppressed
            last_req        = total_req

            status = "DROPPING" if active else "RESUMED "
            remaining_str = f"{remaining:5d}ms left" if active else "         "

            print(f"[{elapsed:6.2f}s] {status} | {remaining_str} | "
                  f"suppressed_total={suppressed:4d} (+{new_suppressed}) | "
                  f"depth={depth_ft} ft")

            if not active and elapsed > (args.ms / 1000 + 0.5):
                # Window has expired and we've confirmed it
                print()
                print(f"[{elapsed:6.2f}s] Drop window ended. Responses resumed.")
                # Poll a few more seconds to see recovery
                for _ in range(int(5.0 / args.poll)):
                    time.sleep(args.poll)
                    elapsed = time.time() - t_start
                    data = fetch_data(args.host)
                    if data:
                        print(f"[{elapsed:6.2f}s] RESUMED  | depth={data.get('depth_ft','?')} ft | "
                              f"rs485_req={data.get('rs485_req','?')}")
                break

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(f"\n[{elapsed:6.2f}s] Interrupted — cancelling drop window.")
        cancel_drop(args.host)

    print()
    print("="*60)
    print("TEST COMPLETE")
    print(f"  Window duration : {args.ms} ms")
    print(f"  Polls suppressed: {last_suppressed - pre_drop}")
    print()
    print("Fill in your observations:")
    print("  T_blink  (display started blinking): ______ s after ENTER")
    print("  T_blank  (display went blank)       : ______ s after ENTER")
    print("  T_recover (solid depth returned)    : ______ s after ENTER")
    print("="*60)


if __name__ == '__main__':
    main()
