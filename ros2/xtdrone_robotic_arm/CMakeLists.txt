cmake_minimum_required(VERSION 3.16)
project(xtdrone_robotic_arm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(vision_manager src/vision_manager.cpp)
ament_target_dependencies(vision_manager rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(vision_manager ${OpenCV_LIBS})

add_executable(grasping_demo src/grasping_demo.cpp)
ament_target_dependencies(grasping_demo rclcpp std_msgs geometry_msgs moveit_ros_planning_interface
  controller_manager_msgs)

install(TARGETS vision_manager grasping_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf meshes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_iris_arm_model.py DESTINATION lib/${PROJECT_NAME})
# the meshes for Gazebo: model://xtdrone_robotic_arm/meshes/... (model iris_arm in xtdrone_gz)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
