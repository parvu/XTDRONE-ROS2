cmake_minimum_required(VERSION 3.16)
project(xtdrone_gz NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds px4 scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Worlds and models converted from Gazebo Classic by scripts/build_xtdrone_ros2.sh
# (classic_to_gz.py). They are generated, not committed.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/worlds")
  install(DIRECTORY generated/worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/usv")
  install(DIRECTORY generated/usv/ DESTINATION share/${PROJECT_NAME}/usv)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/models")
  install(DIRECTORY generated/models/ DESTINATION share/${PROJECT_NAME}/classic_models)
endif()

# Make the XTDrone models and worlds visible to Gazebo Harmonic.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
