# Published by the board on /bhl/board_state at a few Hz.
uint8 STATE_IDLE=0        # outputs off, motors read-only
uint8 STATE_DAMPING=1     # motors in firmware damping
uint8 STATE_ACTIVE=2      # position control, following /lowcmd
uint8 STATE_ESTOP=3       # latched; reset_estop then enable

uint32 FAULT_LOST=1           # no reply from the motor
uint32 FAULT_OUT_OF_RANGE=2   # measured position outside URDF limits at enable
uint32 FAULT_BUS=4            # its CAN bus went away
uint32 FAULT_FIRMWARE=8       # motor reported an error code
uint32 FAULT_CLAMPED=16       # last command was clipped by a limit (info)

uint8 state
string state_name
string command_source     # "none", or the node currently publishing /lowcmd
float32 command_age       # [s] since the last accepted /lowcmd
uint32 rejected_crc
uint32 rejected_other
float32 loop_rate         # [Hz] measured
float32 loop_overrun_max  # [ms] worst late cycle in the last report period
BusState[] buses
string[] faults           # human-readable, one line per problem
