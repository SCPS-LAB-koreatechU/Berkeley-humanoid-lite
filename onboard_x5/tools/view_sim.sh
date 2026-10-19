#!/usr/bin/env bash
# Watch the whole chain in RViz2: PC MoveIt (onboard:=true) -> bhl_commander ->
# bhl_lowlevel -> (simulated) motors -> /bhl/joint_states -> RViz robot model.
# What RViz shows is what the board reports, not what MoveIt planned.
#
#   tools/view_sim.sh                      board in sim on this PC, then a demo
#   tools/view_sim.sh --board sunrise@192.168.1.225   board in sim on the X5
#   tools/view_sim.sh --no-demo            just bring everything up (drag in RViz)
#   Ctrl+C stops everything (board motors -> IDLE).
set -e
cd "$(dirname "$0")/.."
BOARD_WS=$PWD
PC_WS=${PC_WS:-$HOME/Berkeley-Humanoid-Lite/ros2_ws}
board="" demo=1
while [ $# -gt 0 ]; do
  case "$1" in
    --board) board=$2; shift 2 ;;
    --no-demo) demo=0; shift ;;
    *) echo "unknown argument $1" >&2; exit 1 ;;
  esac
done
export ROS_DOMAIN_ID=${BHL_ROS_DOMAIN_ID:-97}     # private domain: never a real robot by accident
tmp=$(mktemp -d)
source /opt/ros/humble/setup.bash; source "$BOARD_WS/install/setup.bash"

pids=()
cleanup() {
  echo; echo "stopping ..."
  for p in "${pids[@]}"; do kill -TERM -- -"$p" 2>/dev/null || true; done
  if [ -n "$board" ]; then
    ssh -o BatchMode=yes "$board" 'p=$(cat /tmp/bhl_view.pid 2>/dev/null) && kill -TERM -- -$p 2>/dev/null; rm -f /tmp/bhl_view.pid' || true
  fi
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ -n "$board" ]; then
  echo "board: sim on $board (domain $ROS_DOMAIN_ID)"
  # The braces keep the backgrounded command alone with its redirections; a
  # backgrounded `cd && ...` list holds the ssh channel open and ssh never returns.
  ssh -o BatchMode=yes "$board" "cd ~/bhl_onboard_ws && { BHL_ROS_DOMAIN_ID=$ROS_DOMAIN_ID PYTHONUNBUFFERED=1 setsid ./run_onboard.sh sim:=true > /tmp/bhl_view.log 2>&1 < /dev/null & echo \$! > /tmp/bhl_view.pid; }"
else
  echo "board: sim on this PC (domain $ROS_DOMAIN_ID)"
  PYTHONUNBUFFERED=1 setsid ros2 launch bhl_onboard onboard.launch.py sim:=true > "$tmp/board.log" 2>&1 < /dev/null &
  pids+=($!)
fi

echo "PC: MoveIt + RViz2 (onboard:=true)"
( source "$PC_WS/install/setup.bash"
  [ -f "$HOME/Desktop/dexhand_moveit_ws/install/setup.bash" ] && source "$HOME/Desktop/dexhand_moveit_ws/install/setup.bash"
  PYTHONUNBUFFERED=1 exec setsid ros2 launch berkeley_humanoid_lite_moveit_config demo.launch.py \
      model:=v1arm onboard:=true ) > "$tmp/pc.log" 2>&1 < /dev/null &
pids+=($!)

for _ in $(seq 90); do grep -q "You can start planning now" "$tmp/pc.log" && break; sleep 0.5; done
grep -q "You can start planning now" "$tmp/pc.log" || { echo "move_group did not start:"; tail -20 "$tmp/pc.log"; exit 1; }
for _ in $(seq 30); do ros2 service list 2>/dev/null | grep -q /bhl_lowlevel/enable && break; sleep 0.5; done
echo "enable: $(ros2 service call /bhl_lowlevel/enable std_srvs/srv/SetBool '{data: true}' | grep -o "message=.*" | cut -c1-70)"

source "$PC_WS/install/setup.bash"
move() {   # group target -- one named target through MoveIt, shown in RViz
  printf "  %-14s -> %-6s " "$1" "$2"
  out=$(timeout 60 ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py "$1" "$2" 2>&1 | tail -1)
  echo "$out" | grep -o "done.*\|fail.*\|abort.*\|rror.*" | head -1 || echo "?"
  sleep 1
}
if [ $demo -eq 1 ]; then
  sleep 3     # let RViz finish loading the model
  echo "demo (watch RViz):"
  move left_arm ready
  move right_arm ready
  move left_shoulder home
  move left_arm home
  move right_arm home
  if [ -n "$board" ]; then
    echo "board side ($board) -- what the X5 received and did:"
    ssh -o BatchMode=yes "$board" "grep -E 'goal (received|done)|ACTIVE|ignoring' /tmp/bhl_view.log | sed -E 's/^\[(bhl_[a-z]+)-[0-9]\] \[INFO\] \[([0-9]+)\.[0-9]+\] \[[a-z_]+\]:/  \1/' | tail -14"
  fi
  echo "  note: right elbow pitch (CAN 8) is enabled: false in bhl_x5.yaml -- the board"
  echo "        leaves it where it is, so it does not bend even though MoveIt planned it."
fi
echo
echo "running. In RViz: drag the interactive marker, Plan & Execute -> the board executes it."
echo "board status:  ros2 topic echo /bhl/board_state   (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "Ctrl+C to stop."
wait
