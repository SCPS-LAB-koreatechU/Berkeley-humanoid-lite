# source this:  source ~/bhl_onboard_ws/env.sh
# Same ROS_DOMAIN_ID as the PC side (Berkeley-Humanoid-Lite/ros2_ws/run_demo.sh uses 77).
# Both machines must also use the same RMW; set RMW_IMPLEMENTATION on both if you change it.
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/tros/humble/setup.bash ]; then     # RDK OS ships TogetheROS (Humble-compatible)
  source /opt/tros/humble/setup.bash
else
  echo "env.sh: no ROS 2 Humble found (/opt/ros/humble or /opt/tros/humble)" >&2
fi
[ -f "$_ws/install/setup.bash" ] && source "$_ws/install/setup.bash"
# Deliberately overrides a ROS_DOMAIN_ID from ~/.bashrc (the X5's says 10, for the
# ZED work): board and PC must agree. Use BHL_ROS_DOMAIN_ID to pick another.
export ROS_DOMAIN_ID="${BHL_ROS_DOMAIN_ID:-77}"
unset _ws
