string name               # e.g. can0
bool present              # interface exists
bool up                   # link is up
bool open                 # the board holds a socket on it
string controller_state   # ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF / ...
bool quiet                # nothing answered for a while -> sending is paused
uint32 tx_dropped         # frames the kernel refused (ENOBUFS)
uint32 error_frames
