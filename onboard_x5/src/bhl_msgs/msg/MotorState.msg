# One actuator's state. Mirrors unitree_hg MotorState_.
#
# mode: 0 = not present (no config entry, or never answered)
#       1 = idle / output off (read-only)
#       2 = damping
#       3 = position control (energised)
uint8 mode
float32 q            # [rad]   URDF joint space
float32 dq           # [rad/s] URDF joint space
float32 tau_est      # [N.m]   reserved, 0 until the firmware reports it
float32 temperature  # [deg C] reserved
uint32 error         # bitfield, see BoardState.FAULT_*
uint32 lost          # consecutive cycles without a reply
uint32 reserve
