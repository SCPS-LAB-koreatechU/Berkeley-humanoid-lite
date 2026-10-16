# One actuator command. Mirrors unitree_hg MotorCmd_ (q, dq, tau, kp, kd).
#
# mode: 0 = leave this motor alone (it keeps its last target while the board
#           is active; it is never energised by this command alone)
#       1 = command it with the fields below
#
# Units are URDF joint space: q [rad], dq [rad/s]. The board converts them with
# each joint's direction / gear_ratio / offset before they reach the motor.
#
# kp / kd / tau go to the recoil firmware as parameters (SDO), not in the fast
# position frame, so the board only rewrites them when they change and at most
# a few times a second. They are in FIRMWARE units -- the same numbers as the
# `gains` block of the board config -- and are capped by that config.
# Set kp = kd = 0 to keep the board's configured gains.
uint8 mode
float32 q
float32 dq
float32 tau
float32 kp
float32 kd
uint32 reserve
