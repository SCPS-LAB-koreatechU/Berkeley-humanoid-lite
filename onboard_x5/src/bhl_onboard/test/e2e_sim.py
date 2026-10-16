"""End-to-end check against the running sim launch (run by ../../test.sh).

Plays the PC's part: enable over the service, send a FollowJointTrajectory goal
to /arms_controller exactly as MoveIt's simple controller manager would, watch
/joint_states, fire a hand preset, check a second /lowcmd publisher is refused,
then E-STOP.
"""

import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from bhl_msgs.msg import BoardState, LowCmd
from bhl_msgs.srv import HandPreset

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""), flush=True)


class Pc(Node):
    def __init__(self):
        super().__init__("e2e_pc")
        self.js = {}
        self.board = None
        self.create_subscription(JointState, "/bhl/joint_states", self._js, 10)
        self.create_subscription(BoardState, "/bhl/board_state", self._bs, 10)

    def _js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def _bs(self, msg):
        self.board = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def call(self, srv_type, name, request, timeout=5.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()


def main():
    rclpy.init()
    pc = Pc()
    check("board state published", pc.wait(lambda: pc.board is not None, 15.0))
    check("joint states published", pc.wait(lambda: "arm_left_shoulder_yaw_joint" in pc.js, 5.0),
          f"{len(pc.js)} joints")
    pc.spin_for(1.0)
    check("loop runs near 200 Hz", pc.board.loop_rate > 150, f"{pc.board.loop_rate:.0f} Hz, "
          f"worst late {pc.board.loop_overrun_max:.2f} ms")

    # trajectory before enable must be rejected
    client = ActionClient(pc, FollowJointTrajectory, "/arms_controller/follow_joint_trajectory")
    check("action server up", client.wait_for_server(timeout_sec=5.0))

    def goal(points, names):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = names
        for t, q in points:
            p = JointTrajectoryPoint()
            p.positions = q
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            g.trajectory.points.append(p)
        return g

    names = ["arm_left_shoulder_yaw_joint", "arm_left_elbow_pitch_joint",
             "arm_right_shoulder_yaw_joint", "arm_left_wrist_yaw_joint"]   # wrist: unmapped
    send = client.send_goal_async(goal([(1.0, [0.3, 0.6, -0.3, 0.0])], names))
    rclpy.spin_until_future_complete(pc, send, timeout_sec=5.0)
    check("goal rejected while IDLE", not send.result().accepted)

    res = pc.call(SetBool, "/bhl_lowlevel/enable", SetBool.Request(data=True))
    check("enable", res is not None and res.success, res.message if res else "no service")
    check("board ACTIVE", pc.wait(lambda: pc.board.state == BoardState.STATE_ACTIVE, 3.0))
    pc.spin_for(0.5)

    start = dict(pc.js)
    send = client.send_goal_async(goal([(1.0, [0.3, 0.6, -0.3, 0.0]),
                                        (2.0, [0.4, 0.9, -0.4, 0.0])], names))
    rclpy.spin_until_future_complete(pc, send, timeout_sec=5.0)
    handle = send.result()
    check("goal accepted", handle.accepted)
    t0 = time.monotonic()
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(pc, result_future, timeout_sec=10.0)
    elapsed = time.monotonic() - t0
    result = result_future.result().result
    check("goal succeeded", result.error_code == 0, f"code {result.error_code} '{result.error_string}' "
          f"after {elapsed:.2f} s")
    pc.spin_for(0.3)
    err = max(abs(pc.js[n] - q) for n, q in zip(names[:3], [0.4, 0.9, -0.4]))
    check("joints reached the final point", err < 0.03, f"max error {err:.4f} rad, "
          f"left yaw {start['arm_left_shoulder_yaw_joint']:+.3f} -> {pc.js['arm_left_shoulder_yaw_joint']:+.3f}")
    check("ran on schedule", 1.9 < elapsed < 3.0, f"{elapsed:.2f} s for a 2.0 s trajectory")

    # out-of-limit goal is refused, joint does not move
    send = client.send_goal_async(goal([(1.0, [2.5, 0.6, -0.3, 0.0])], names))
    rclpy.spin_until_future_complete(pc, send, timeout_sec=5.0)
    rf = send.result().get_result_async()
    rclpy.spin_until_future_complete(pc, rf, timeout_sec=5.0)
    check("out-of-limit goal aborted", rf.result().result.error_code ==
          FollowJointTrajectory.Result.INVALID_GOAL, rf.result().result.error_string)

    # hand preset (config used by test.sh sets the left hand to driver: sim)
    res = pc.call(HandPreset, "/bhl/hand_preset", HandPreset.Request(side="left", name="test_close",
                                                                     duration=0.5))
    check("hand preset accepted", res is not None and res.success, res.message if res else "")
    ok = pc.wait(lambda: abs(pc.js.get("left_hand_m05", 0.0) - 0.5) < 0.02, 3.0)
    check("hand reached preset", ok, f"left_hand_m05 = {pc.js.get('left_hand_m05', float('nan')):+.3f}")

    # a second /lowcmd publisher -> the board stops accepting and says so
    rogue = pc.create_publisher(LowCmd, "/lowcmd", 1)
    blocked = pc.wait(lambda: any("multiple /lowcmd" in f for f in pc.board.faults), 3.0)
    check("second /lowcmd publisher detected", blocked)
    pc.destroy_publisher(rogue)
    check("cleared once it leaves",
          pc.wait(lambda: not any("multiple" in f for f in pc.board.faults), 4.0))
    check("still ACTIVE (hold window not exceeded)", pc.board.state == BoardState.STATE_ACTIVE,
          pc.board.state_name)

    res = pc.call(Trigger, "/bhl_lowlevel/estop", Trigger.Request())
    check("estop", res is not None and res.success)
    check("board ESTOP", pc.wait(lambda: pc.board.state == BoardState.STATE_ESTOP, 2.0))
    res = pc.call(SetBool, "/bhl_lowlevel/enable", SetBool.Request(data=True))
    check("enable refused while latched", res is not None and not res.success, res.message if res else "")
    res = pc.call(Trigger, "/bhl_lowlevel/reset_estop", Trigger.Request())
    check("reset_estop", res is not None and res.success)

    pc.destroy_node()
    rclpy.shutdown()
    print(f"\n{sum(RESULTS)}/{len(RESULTS)} PASS")
    sys.exit(0 if all(RESULTS) else 1)


if __name__ == "__main__":
    main()
