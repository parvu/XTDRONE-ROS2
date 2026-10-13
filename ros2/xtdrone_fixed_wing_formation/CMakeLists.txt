cmake_minimum_required(VERSION 3.16)
project(xtdrone_fixed_wing_formation)

# ROS 2 port of XTDrone's coordination/fixed_wing_formation_control (chapter 35): fixed-wing formation
# following a leader that loiters over a UGV. The formation controller and TECS are the ROS 1 sources.
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FormationControlStates.msg msg/FwCmdMode.msg msg/FwCmd.msg msg/FwCurrentMode.msg msg/FwMonitor.msg
  msg/FwStates.msg msg/LeaderStates.msg
  DEPENDENCIES geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(formation_controller
  src/formation_controller/abs_formation_controller.cpp
  src/formation_controller/formation_controller.cpp
  src/fixed_wing_lib/vertical_controller/tecs.cpp)

foreach(node pack_fw_states task_main vir_sim_leader switch_fw_mode)
  add_executable(${node} src/nodes/${node}.cpp)
  ament_target_dependencies(${node} rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs mavros_msgs)
  target_link_libraries(${node} "${cpp_typesupport_target}")
endforeach()
target_link_libraries(task_main formation_controller)

install(TARGETS pack_fw_states task_main vir_sim_leader switch_fw_mode DESTINATION lib/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
