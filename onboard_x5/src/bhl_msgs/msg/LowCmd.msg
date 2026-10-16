# Whole-body low-level command, published by ONE commander at a time on /lowcmd.
# Mirrors unitree_hg LowCmd_: fixed-size motor array, index = motor slot.
#
# Slot table (BHL order, arms first as in berkeley_humanoid_lite_lowlevel
# robot/humanoid.py): see config/bhl_x5.yaml `motors` for the authoritative list.
#   0..4   left arm   (CAN ids 1 3 5 7 9)
#   5..9   right arm  (CAN ids 2 4 6 8 10)
#   10..15 left leg   (reserved)
#   16..21 right leg  (reserved)
#
# crc: CRC-32 over the packed motor commands (bhl_onboard.crc.lowcmd_crc).
# The board drops a message whose crc does not match, so a stale or
# half-built command is never executed. Clients fill it last.
uint8 NUM_MOTOR=22

uint8 mode_machine        # robot variant, reserved
MotorCmd[22] motor_cmd
uint32[4] reserve
uint32 crc
