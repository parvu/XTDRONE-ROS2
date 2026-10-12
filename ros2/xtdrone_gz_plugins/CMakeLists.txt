cmake_minimum_required(VERSION 3.16)
project(xtdrone_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Gazebo Harmonic systems of XTDrone (loaded with filename="xtdrone_actor_motion").
add_library(xtdrone_actor_motion SHARED src/ActorMotion.cc)
target_link_libraries(xtdrone_actor_motion
  gz-plugin2::register gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
install(TARGETS xtdrone_actor_motion LIBRARY DESTINATION lib/${PROJECT_NAME})

# GZ_SIM_SYSTEM_PLUGIN_PATH for gz sim
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
