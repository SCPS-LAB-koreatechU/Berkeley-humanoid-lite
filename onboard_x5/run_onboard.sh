#!/usr/bin/env bash
# Start the on-board controller.
#   ./run_onboard.sh                 live (RDK X5, CAN up)
#   ./run_onboard.sh sim:=true       fake CAN, any machine
#   ./run_onboard.sh commander:=false   only bhl_lowlevel (drive /lowcmd yourself)
#   ./run_onboard.sh config:=/path/to/other.yaml
set -e
cd "$(dirname "$0")"
source ./env.sh
if [[ " $* " != *" sim:=true "* ]]; then
  for ch in $(python3 - <<'PY'
import yaml; c = yaml.safe_load(open("src/bhl_onboard/config/bhl_x5.yaml"))
print(" ".join(sorted(set((c.get("buses") or {}).values()))))
PY
  ); do
    if [ ! -e "/sys/class/net/$ch" ]; then
      echo "warning: $ch missing (adapter unplugged?) -- the board will attach it when it appears"
    elif ! ip link show "$ch" | grep -q "state UP\|,UP"; then
      echo "warning: $ch is down. Bring it up (two separate commands):"
      echo "  sudo ip link set $ch down"
      echo "  sudo ip link set $ch up type can bitrate 1000000"
    fi
  done
fi
exec ros2 launch bhl_onboard onboard.launch.py "$@"
