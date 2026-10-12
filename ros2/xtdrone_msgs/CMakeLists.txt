cmake_minimum_required(VERSION 3.8)
project(xtdrone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Messages of coordination/formation_communication_demo/control (chapter 27, XTDGroundControl).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UAVDataMsg.msg"
  "msg/AllUAVData.msg"
  "msg/UavComm.msg"
  "msg/CommVerify.msg"
  "msg/NeighborMsg.msg"
  "msg/AllNeighborMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
