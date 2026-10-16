cmake_minimum_required(VERSION 3.8)
project(bhl_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/IMUState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/HandCmd.msg"
  "msg/HandState.msg"
  "msg/BusState.msg"
  "msg/BoardState.msg"
  "srv/HandPreset.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
