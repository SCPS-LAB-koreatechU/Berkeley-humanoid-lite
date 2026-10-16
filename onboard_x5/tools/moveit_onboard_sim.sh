#!/usr/bin/env bash
# PC + board together on one machine: bhl_onboard in sim and the PC MoveIt demo in
# onboard mode (headless) on a private domain, then motion_commander plans and
# executes named targets through the board. Exercises exactly the path the real
# robot uses, minus CAN and Wi-Fi.
#   tools/moveit_onboard_sim.sh [group target ...]   default: left_arm ready left_arm home
set -e
cd "$(dirname "$0")/.."
BOARD_WS=$PWD
PC_WS=${PC_WS:-$HOME/Berkeley-Humanoid-Lite/ros2_ws}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-98}
tmp=$(mktemp -d); export ROS_LOG_DIR="$tmp/roslog"
targets=("$@"); [ ${#targets[@]} -eq 0 ] && targets=(left_arm ready left_arm home)

source /opt/ros/humble/setup.bash; source "$BOARD_WS/install/setup.bash"
PYTHONUNBUFFERED=1 setsid ros2 launch bhl_onboard onboard.launch.py sim:=true > "$tmp/board.log" 2>&1 < /dev/null &
board=$!
( source "$PC_WS/install/setup.bash"; [ -f "$HOME/Desktop/dexhand_moveit_ws/install/setup.bash" ] && source "$HOME/Desktop/dexhand_moveit_ws/install/setup.bash"
  PYTHONUNBUFFERED=1 exec setsid ros2 launch berkeley_humanoid_lite_moveit_config demo.launch.py \
      model:=v1arm onboard:=true use_rviz:=false ) > "$tmp/pc.log" 2>&1 < /dev/null &
pc=$!
cleanup() {
  kill -TERM -- -$pc -$board 2>/dev/null; wait $pc $board 2>/dev/null
  for _ in $(seq 50); do pgrep -g $board >/dev/null || pgrep -g $pc >/dev/null || break; sleep 0.2; done
}
trap cleanup EXIT

for _ in $(seq 60); do grep -q "You can start planning now" "$tmp/pc.log" && break; sleep 0.5; done
grep -q "You can start planning now" "$tmp/pc.log" || { echo "move_group did not come up"; tail -20 "$tmp/pc.log"; exit 1; }
ros2 service call /bhl_lowlevel/enable std_srvs/srv/SetBool '{data: true}' | grep -o "success=[A-Za-z]*.*" | cut -c1-80
source "$PC_WS/install/setup.bash"
rc=0
set -- "${targets[@]}"
while [ $# -ge 2 ]; do
  out=$(timeout 60 ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py "$1" "$2" 2>&1) || rc=1
  echo "$1 -> $2: $(echo "$out" | grep -v "^\s*$" | tail -1 | cut -c1-140)"
  python3 - "$1" "$2" <<'PY' || rc=1
import sys, time, rclpy, xml.etree.ElementTree as ET
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters
group, target = sys.argv[1:3]
rclpy.init(); n = rclpy.create_node("verify")
c = n.create_client(GetParameters, "/move_group/get_parameters"); c.wait_for_service(timeout_sec=5)
f = c.call_async(GetParameters.Request(names=["robot_description_semantic"])); rclpy.spin_until_future_complete(n, f, timeout_sec=5)
srdf = ET.fromstring(f.result().values[0].string_value)
want = {j.get("name"): float(j.get("value")) for gs in srdf.findall("group_state")
        if gs.get("name") == target and gs.get("group") == group for j in gs.findall("joint")}
got = {}
n.create_subscription(JointState, "/bhl/joint_states", lambda m: got.update(zip(m.name, m.position)), 10)
end = time.time() + 3
while time.time() < end and not got: rclpy.spin_once(n, timeout_sec=0.1)
rows = [(k, v, got[k]) for k, v in want.items() if k in got]
err = max(abs(min(max(v, -9), 9) - g) for _, v, g in rows)
print("   board measured: " + ", ".join(f"{k.replace('arm_left_', 'L.').replace('arm_right_', 'R.').replace('_joint', '')} {g:+.3f} (goal {v:+.2f})" for k, v, g in rows))
ok = all(abs(v - g) <= 0.06 for _, v, g in rows)   # board stops 0.05 inside a URDF limit
print(f"   [{'PASS' if ok else 'FAIL'}] board joints at the named target (max |err| {err:.3f} rad)")
sys.exit(0 if ok else 1)
PY
  shift 2
done
grep -h "ignoring joints\|rejected\|GOAL_TOLERANCE\|final error" "$tmp"/roslog/*.log 2>/dev/null | sed 's/^/   board: /' | cut -c1-160 | sort -u | head -5
exit $rc
