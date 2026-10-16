#!/usr/bin/env bash
# One-time setup ON THE RDK X5 (run as the normal user; it asks for sudo).
#   ./setup_x5.sh              packages + CAN bring-up at boot + build
#   ./setup_x5.sh --autostart  also start bhl_onboard at boot (it starts in IDLE:
#                              nothing moves until someone calls enable)
#
# UNVERIFIED on a real X5 so far (written 2026-10-16 without a board). Things to
# check the first time: that ROS 2 Humble is /opt/ros/humble or /opt/tros/humble,
# that the kernel has gs_usb (USB-CAN adapters such as candleLight/canable), and
# which canX names the adapters get.
set -e
cd "$(dirname "$0")"
WS=$PWD

echo "== system"
uname -a
. /etc/os-release; echo "$PRETTY_NAME"
[ "$VERSION_ID" = "22.04" ] || echo "warning: expected Ubuntu 22.04 (RDK OS 3.x), got $VERSION_ID"

echo "== ROS 2"
if [ -f /opt/ros/humble/setup.bash ]; then ROS=/opt/ros/humble
elif [ -f /opt/tros/humble/setup.bash ]; then ROS=/opt/tros/humble
else
  echo "no ROS 2 Humble. Install ros-humble-ros-base from packages.ros.org first."; exit 1
fi
echo "using $ROS"
pkgs="python3-can python3-yaml python3-pytest can-utils python3-colcon-common-extensions"
# control_msgs (FollowJointTrajectory) is not part of every TROS image
source "$ROS/setup.bash"
python3 -c "import control_msgs" 2>/dev/null || pkgs="$pkgs ros-humble-control-msgs"
python3 -c "import std_srvs, sensor_msgs" 2>/dev/null || pkgs="$pkgs ros-humble-std-srvs ros-humble-sensor-msgs"
echo "installing: $pkgs"
sudo apt-get update
sudo apt-get install -y $pkgs

echo "== CAN"
if modinfo gs_usb >/dev/null 2>&1; then echo "gs_usb driver available"; else
  echo "warning: kernel has no gs_usb module -- USB-CAN adapters will not appear as canX"; fi
ip -br link show type can || true
# Bring every CAN interface up at 1 Mbit/s whenever it appears (boot or re-plug).
# Note: gs_usb does not support restart-ms; do not add it (the command then fails
# and the link stays down).
sudo tee /etc/udev/rules.d/80-bhl-can.rules >/dev/null <<'RULE'
SUBSYSTEM=="net", ACTION=="add", KERNEL=="can[0-9]*", RUN+="/usr/local/sbin/bhl-can-up %k"
RULE
sudo tee /usr/local/sbin/bhl-can-up >/dev/null <<'SH'
#!/bin/sh
# $1 = can0, can1, ...
/sbin/ip link set "$1" down 2>/dev/null
/sbin/ip link set "$1" type can bitrate 1000000
/sbin/ip link set "$1" txqueuelen 100
/sbin/ip link set "$1" up
SH
sudo chmod +x /usr/local/sbin/bhl-can-up
sudo udevadm control --reload
for ch in $(ls /sys/class/net | grep -E '^can[0-9]+$' || true); do sudo /usr/local/sbin/bhl-can-up "$ch"; done
ip -details -br link show type can || true

echo "== build"
source "$ROS/setup.bash"
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q src/bhl_onboard/test/test_core.py

if [ "$1" = "--autostart" ]; then
  echo "== autostart service"
  sudo tee /etc/systemd/system/bhl-onboard.service >/dev/null <<UNIT
[Unit]
Description=Berkeley Humanoid Lite on-board controller (starts IDLE)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$WS
ExecStart=/bin/bash -lc 'exec $WS/run_onboard.sh'
KillSignal=SIGINT
TimeoutStopSec=10
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
  sudo systemctl daemon-reload
  sudo systemctl enable --now bhl-onboard.service
  systemctl --no-pager status bhl-onboard.service | head -5
fi
echo
echo "done. Next: tools/bench_sim.sh (can this board keep 200 Hz?), then ./run_onboard.sh"
