# Whole-body low-level state, published by the board on /lowstate.
# Mirrors unitree_hg LowState_. Slots as in LowCmd.
uint32 tick               # control cycles since the board started
uint8 mode_machine
uint8 board_state         # BoardState.STATE_*
IMUState imu_state
MotorState[22] motor_state
uint32[4] reserve
uint32 crc
