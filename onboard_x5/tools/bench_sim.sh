#!/usr/bin/env bash
# Can this machine keep the loop rate? Runs the sim launch, enables, streams a
# trajectory through the commander, and reports per-node CPU, loop rate and the
# worst late cycle. Run it on the RDK X5 before connecting motors.
#   tools/bench_sim.sh [seconds]
#   BENCH_CONFIG=/path/to.yaml tools/bench_sim.sh   # another config (rates, buses)
set -e
cd "$(dirname "$0")/.."
source ./env.sh
export ROS_DOMAIN_ID=96
secs=${1:-15}
tmp=$(mktemp -d)
PYTHONUNBUFFERED=1 setsid ros2 launch bhl_onboard onboard.launch.py sim:=true ${BENCH_CONFIG:+config:=$BENCH_CONFIG} > "$tmp/launch.log" 2>&1 < /dev/null &
lp=$!
cleanup() {
  kill -TERM -- -$lp 2>/dev/null; wait $lp 2>/dev/null
  for _ in $(seq 50); do pgrep -g $lp >/dev/null || break; sleep 0.2; done
}
trap cleanup EXIT
sleep 4
ros2 service call /bhl_lowlevel/enable std_srvs/srv/SetBool '{data: true}' > /dev/null
python3 - "$secs" <<'PY'
import os, sys, time, glob, math, threading
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bhl_msgs.msg import BoardState

secs = float(sys.argv[1])
def pids(name):
    out = []
    for d in glob.glob("/proc/[0-9]*"):
        try:
            if name in open(f"{d}/cmdline").read():
                out.append(int(d[6:]))
        except OSError:
            pass
    return out
def ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])
hz = os.sysconf("SC_CLK_TCK")
procs = {n: pids(f"lib/bhl_onboard/{n}") for n in ("bhl_lowlevel", "bhl_commander")}

rclpy.init(); node = rclpy.create_node("bench")
states = []
node.create_subscription(BoardState, "/bhl/board_state", states.append, 10)
client = ActionClient(node, FollowJointTrajectory, "/arms_controller/follow_joint_trajectory")
client.wait_for_server(timeout_sec=5)
names = ["arm_left_shoulder_yaw_joint", "arm_right_shoulder_yaw_joint", "arm_left_elbow_roll_joint"]
g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = names
for k in range(1, int(secs * 2)):
    p = JointTrajectoryPoint(); a = 0.3 * math.sin(k * 0.5)
    p.positions = [a, -a, a]; p.time_from_start = Duration(sec=k // 2, nanosec=(k % 2) * 500000000)
    g.trajectory.points.append(p)
client.send_goal_async(g)
start = {n: [ticks(p) for p in ps] for n, ps in procs.items()}
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.1)
dt = time.monotonic() - t0
for n, ps in procs.items():
    used = sum(ticks(p) for p in ps) - sum(start[n])
    print(f"{n:14s} CPU {100 * used / hz / dt:5.1f}% of one core")
active = [s for s in states if s.state_name == "ACTIVE"]
if active:
    rates = [s.loop_rate for s in active]
    print(f"loop rate      min {min(rates):.0f} Hz, mean {sum(rates)/len(rates):.0f} Hz")
    print(f"worst late     {max(s.loop_overrun_max for s in active):.1f} ms "
          f"(at the configured control.rate_hz)")
    print(f"faults         {sorted(set(f for s in active for f in s.faults)) or 'none'}")
node.destroy_node(); rclpy.shutdown()
PY
