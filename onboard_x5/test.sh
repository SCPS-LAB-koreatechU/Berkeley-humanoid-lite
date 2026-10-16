#!/usr/bin/env bash
# Build, unit tests, then an end-to-end run of the sim launch with the PC's part
# played by test/e2e_sim.py. Uses its own ROS_DOMAIN_ID so it never talks to a
# real robot.
set -e
cd "$(dirname "$0")"
source ./env.sh
colcon build --symlink-install 2>&1 | grep -E "Finished|Failed|error" || true
source install/setup.bash
export ROS_DOMAIN_ID=91

echo "== unit tests"
python3 -m pytest -q src/bhl_onboard/test/test_core.py

echo "== end-to-end (sim)"
tmp=$(mktemp -d)
python3 - "$tmp/e2e.yaml" <<'PY'
import sys, yaml
c = yaml.safe_load(open("src/bhl_onboard/config/bhl_x5.yaml"))
c["hands"]["left"]["driver"] = "sim"
c["hand_presets"]["test_close"] = [0.0] * 5 + [0.5] + [0.0] * 11
yaml.safe_dump(c, open(sys.argv[1], "w"))
PY
export BHL_OFFSETS_FILE="$tmp/offsets.yaml"
export ROS_LOG_DIR="$tmp/roslog"   # node log files survive the launch being killed
PYTHONUNBUFFERED=1 setsid ros2 launch bhl_onboard onboard.launch.py sim:=true config:="$tmp/e2e.yaml" > "$tmp/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- -"$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  # launch can exit before its nodes have put the motors to IDLE: wait for them
  for _ in $(seq 50); do pgrep -g "$launch_pid" >/dev/null || break; sleep 0.2; done
}
trap cleanup EXIT
set +e
python3 src/bhl_onboard/test/e2e_sim.py
rc=$?
cleanup
trap - EXIT
if grep -rq "shutting down: motors -> IDLE" "$tmp/roslog" "$tmp/launch.log"; then
  echo "[PASS] clean shutdown (motors -> IDLE)"
else
  echo "[FAIL] no clean shutdown line"; rc=1
fi
if [ $rc -ne 0 ]; then echo "--- launch log ($tmp/launch.log)"; tail -40 "$tmp/launch.log"; fi
if pgrep -g "$launch_pid" >/dev/null; then echo "[FAIL] node processes still alive after 10 s"; rc=1
else echo "[PASS] no processes left behind"; fi
exit $rc
