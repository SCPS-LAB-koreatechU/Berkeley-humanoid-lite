# One hand+wrist unit (ESP32 node, 17 motors). Mirrors unitree_hg HandCmd_.
# Topics: /hand/left/cmd, /hand/right/cmd. Slot meaning: config `hands`.
uint8 NUM_MOTOR=17
MotorCmd[17] motor_cmd
uint32[4] reserve
