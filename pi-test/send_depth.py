#!/usr/bin/env python3
"""
send_depth.py — Raspberry Pi bench test: inject PGN 128267 (Water Depth)
onto a SocketCAN interface at a fixed interval.

Usage:
    python3 send_depth.py [--depth 8.9] [--iface can0] [--interval 1.0]

Hardware:
    MCP2515/TJA1050 hat configured as can0 at 250 kbps (see README).

PGN 128267 — Water Depth
    CAN extended frame, 29-bit ID.
    Fast-packet protocol: first frame carries PGN, subsequent frames carry data.
    For bench testing we send a single-frame fast-packet (simplest form that
    the ESP32 gateway's TWAI listener accepts — it reads raw CAN frames and
    extracts the depth bytes directly, not a full fast-packet reassembler).

    NMEA 2000 CAN ID construction:
        Priority  : 3 bits  (default 6 = 0b110)
        Reserved  : 1 bit   (0)
        Data page : 1 bit   (0)
        PF        : 8 bits  (PGN 128267 → PF = 0xF5, PDU2 format)
        PS        : 8 bits  (destination, broadcast = 0xFF for PDU2)
        SA        : 8 bits  (source address, use 0x01 for Pi)

    29-bit ID for PGN 128267, priority 6, SA=0x01:
        priority=6  → bits[28:26] = 0b110
        R=0, DP=0   → bits[25:24] = 0b00
        PF=0xF5     → bits[23:16] = 0b11110101
        PS=0xFF     → bits[15:8]  = 0b11111111
        SA=0x01     → bits[7:0]   = 0b00000001

        ID = (6<<26)|(0<<25)|(0<<24)|(0xF5<<16)|(0xFF<<8)|0x01
           = 0x18F5FF01

    Data bytes (8 bytes):
        [0]   SID           — sequence ID, ignored by gateway
        [1-3] Depth         — 24-bit unsigned, 0.01 m/bit
        [4-5] Offset        — 16-bit signed, 0.001 m/bit (set 0)
        [6]   Range         — 0xFF = unknown
        [7]   Padding       — 0xFF

    Depth encoding: depth_m = depth_ft / 3.28084
                    raw = round(depth_m / 0.01) = round(depth_m * 100)

Fast-packet framing (NMEA 2000):
    Byte 0: sequence counter (bits 7:5) + frame counter (bits 4:0)
    Frame 0 (first): frame_counter=0, byte 1 = total data length (7), bytes 2-7 = first 6 data bytes
    Frame 1+: frame_counter=N, bytes 1-7 = next 7 data bytes

    The ESP32 gateway uses ESP32-TWAI-CAN and reads raw CAN frames. The library
    exposes raw payload bytes. Our gateway's handleDepth() reads:
        depth bytes from buf[3..5] (after the 2 fast-packet header bytes + SID)
    So we need proper fast-packet framing.
"""

import argparse
import socket
import struct
import time
import sys

# NMEA 2000 CAN ID for PGN 128267, priority 6, SA=0x01
#
# PGN 128267 = 0x01F50B
#   DP  = (128267 >> 16) & 0x01 = 1
#   PF  = (128267 >>  8) & 0xFF = 0xF5  (245, >= 240 -> PDU2 broadcast)
#   PS  = (128267      ) & 0xFF = 0x0B  (11, part of PGN in PDU2)
#
# 29-bit CAN ID layout: [priority:3][R:1][DP:1][PF:8][PS:8][SA:8]
#   = (6<<26) | (0<<25) | (1<<24) | (0xF5<<16) | (0x0B<<8) | 0x01
#   = 0x19F50B01
#
CAN_ID_PGN128267 = (6 << 26) | (1 << 24) | (0xF5 << 16) | (0x0B << 8) | 0x01
CAN_ID_EFF       = 0x80000000   # extended frame flag (SocketCAN: tells kernel this is a 29-bit ID)
CAN_EFF_ID       = CAN_ID_PGN128267 | CAN_ID_EFF


def depth_to_raw(depth_ft: float) -> int:
    """Convert feet to PGN 128267 raw value (0.01m per bit, 24-bit)."""
    depth_m = depth_ft / 3.28084
    return max(0, min(0xFFFFFF, round(depth_m * 100)))


def build_fast_packet_frames(pgn_data: bytes, seq_id: int = 0) -> list[bytes]:
    """
    Build NMEA 2000 fast-packet CAN frames for up to 223 bytes of PGN data.
    Returns a list of 8-byte CAN payloads (one per frame).

    Frame 0: [seq<<5|0x00] [total_len] [data[0:6]]
    Frame N: [seq<<5|N]    [data[6+7*(N-1) : 6+7*N]] (padded with 0xFF)
    """
    total = len(pgn_data)
    frames = []
    seq = (seq_id & 0x07) << 5

    # Frame 0
    chunk = pgn_data[0:6]
    chunk = chunk.ljust(6, b'\xff')
    frames.append(bytes([seq | 0x00, total]) + chunk)

    # Subsequent frames
    frame_num = 1
    offset = 6
    while offset < total:
        chunk = pgn_data[offset:offset + 7]
        chunk = chunk.ljust(7, b'\xff')
        frames.append(bytes([seq | frame_num]) + chunk)
        offset += 7
        frame_num += 1

    return frames


def build_pgn128267_data(depth_ft: float, sid: int = 0) -> bytes:
    """
    Build the 7-byte PGN 128267 (Water Depth) payload.
    Layout: SID(1) | Depth(3, 0.01m/bit) | Offset(2, 0.001m/bit) | Range(1)
    """
    raw = depth_to_raw(depth_ft)
    d0  = raw & 0xFF
    d1  = (raw >> 8) & 0xFF
    d2  = (raw >> 16) & 0xFF
    return struct.pack('<BBHHHB',
        sid & 0xFF,         # SID
        d0, d1, d2,         # depth 24-bit LE — manual since struct has no 3-byte type
        0x0000,             # offset = 0
        0x00FF,             # range = unknown (0xFF field) + padding
    )[:7]


def pgn128267_bytes(depth_ft: float, sid: int = 0) -> bytes:
    """Return raw 7-byte PGN 128267 payload."""
    raw = depth_to_raw(depth_ft)
    d0  =  raw & 0xFF
    d1  = (raw >> 8)  & 0xFF
    d2  = (raw >> 16) & 0xFF
    # SID(1) | depth[0](1) | depth[1](1) | depth[2](1) | offset_lo(1) | offset_hi(1) | range(1)
    return bytes([sid & 0xFF, d0, d1, d2, 0x00, 0x00, 0xFF])


def open_can_socket(iface: str) -> socket.socket:
    """Open a raw SocketCAN socket bound to the given interface."""
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def send_frame(s: socket.socket, can_id: int, payload: bytes) -> None:
    """Send a single CAN frame (standard SocketCAN struct: id(4) len(1) pad(3) data(8))."""
    payload = payload[:8].ljust(8, b'\xff')
    frame = struct.pack('<IB3x8s', can_id, len(payload), payload)
    s.send(frame)


def build_raw_frame(depth_ft: float, sid: int = 0) -> bytes:
    """
    Build an 8-byte CAN payload that handleDepth() on the ESP32 gateway can
    parse directly.

    The gateway does NOT reassemble fast-packets.  handleDepth() reads:
        rawM = data[1] | data[2]<<8 | data[3]<<16 | data[4]<<24

    So we lay out the PGN 128267 payload with NO fast-packet header:
        data[0] = SID
        data[1] = depth byte 0 (LSB)
        data[2] = depth byte 1
        data[3] = depth byte 2 (MSB, almost always 0)
        data[4] = depth byte 3 (always 0 for depths < 655 m)
        data[5] = offset LSB (0x00)
        data[6] = offset MSB (0x00)
        data[7] = range      (0xFF = unknown)

    With fast-packet framing the first two bytes would be the fast-packet
    header (seq|0x00, total_len=7), which places 0x07 in data[1] and shifts
    all depth bytes two positions right — causing the gateway to read a
    garbage rawM that exceeds the 30000 guard and drops the frame.
    """
    raw = depth_to_raw(depth_ft)
    d0  =  raw        & 0xFF
    d1  = (raw >> 8)  & 0xFF
    d2  = (raw >> 16) & 0xFF
    return bytes([sid & 0xFF, d0, d1, d2, 0x00, 0x00, 0x00, 0xFF])


def main():
    parser = argparse.ArgumentParser(
        description='Inject PGN 128267 (water depth) onto a SocketCAN interface')
    parser.add_argument('--depth',    type=float, default=8.9,   help='Depth in feet (default: 8.9)')
    parser.add_argument('--iface',    type=str,   default='can0', help='SocketCAN interface (default: can0)')
    parser.add_argument('--interval', type=float, default=1.0,   help='Send interval in seconds (default: 1.0)')
    parser.add_argument('--count',    type=int,   default=0,     help='Frames to send (0 = infinite)')
    args = parser.parse_args()

    raw = depth_to_raw(args.depth)
    print(f"[send_depth] iface={args.iface}  depth={args.depth:.1f} ft  "
          f"raw=0x{raw:06X} ({raw})  interval={args.interval:.2f}s")
    print(f"[send_depth] CAN ID=0x{CAN_ID_PGN128267:08X} (PGN 128267, prio=6, SA=0x01)")
    print(f"[send_depth] NOTE: sending raw PGN payload (no fast-packet header) — matches gateway parser")
    print(f"[send_depth] Press Ctrl+C to stop.\n")

    try:
        s = open_can_socket(args.iface)
    except OSError as e:
        print(f"ERROR: cannot open {args.iface}: {e}", file=sys.stderr)
        print("  Is the interface up?  Try: sudo ip link set can0 up type can bitrate 250000",
              file=sys.stderr)
        sys.exit(1)

    sid = 0
    count = 0
    try:
        while args.count == 0 or count < args.count:
            frame = build_raw_frame(args.depth, sid)
            send_frame(s, CAN_EFF_ID, frame)
            sid = (sid + 1) & 0x07
            count += 1
            if count % 10 == 0:
                print(f"  [{count:6d}] sent depth={args.depth:.1f}ft  frame={frame.hex()}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print(f"\n[send_depth] Stopped after {count} transmissions.")
    finally:
        s.close()


if __name__ == '__main__':
    main()
