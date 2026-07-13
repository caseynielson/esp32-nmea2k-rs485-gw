#!/bin/bash
# setup_can.sh — Configure MCP2515 SocketCAN interface on Raspberry Pi
#
# Run once after boot (or add to /etc/rc.local / a systemd unit).
# Requires: can-utils, kernel module mcp251x loaded via /boot/config.txt overlay.
#
# /boot/config.txt entries needed (add if not present, then reboot):
#   dtparam=spi=on
#   dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
#   dtoverlay=spi-bcm2835
#
# Oscillator value depends on your hat:
#   8MHz  → oscillator=8000000   (common on cheap MCP2515 modules)
#   16MHz → oscillator=16000000  (PiCAN2, some others)
# Check your hat's crystal — it's the silver can next to the MCP2515 chip.
#
# Interrupt GPIO: most MCP2515 hats use GPIO25 (pin 22). Check your hat's
# documentation if CAN init fails.

set -e

IFACE=${1:-can0}
BITRATE=250000   # NMEA 2000 standard

echo "[setup_can] Bringing up $IFACE at ${BITRATE} bps..."

# Load module if not already loaded
if ! lsmod | grep -q mcp251x; then
    echo "[setup_can] Loading mcp251x kernel module..."
    sudo modprobe mcp251x
fi

# Bring up the interface
sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" up type can bitrate $BITRATE

echo "[setup_can] $IFACE is up at ${BITRATE} bps."
echo ""
echo "Verify with:"
echo "  ip -details link show $IFACE"
echo "  candump $IFACE"
echo ""
echo "Run depth injector:"
echo "  python3 send_depth.py --depth 8.9 --iface $IFACE"
