cmake_minimum_required(VERSION 3.16)
project(xtdrone_orbslam3)

# ORB-SLAM3 v1.0 (upstream v1.0-release, patched) and Pangolin are built by
# scripts/build_orbslam3.sh into ORB_SLAM3_DIR (default ~/.cache/xtdrone/orbslam3).
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT ORB_SLAM3_DIR)
  if(DEFINED ENV{ORB_SLAM3_DIR})
    set(ORB_SLAM3_DIR $ENV{ORB_SLAM3_DIR})
  else()
    set(ORB_SLAM3_DIR $ENV{HOME}/.cache/xtdrone/orbslam3/ORB_SLAM3)
  endif()
endif()
if(NOT EXISTS ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so)
  message(WARNING "ORB-SLAM3 not built (${ORB_SLAM3_DIR}); run scripts/build_orbslam3.sh. Skipping stereo_node.")
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_package()
  return()
endif()
find_package(Pangolin REQUIRED PATHS $ENV{HOME}/.cache/xtdrone/orbslam3/pangolin-install NO_DEFAULT_PATH)

add_executable(stereo_node src/stereo_node.cpp)
target_include_directories(stereo_node PRIVATE
  ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(stereo_node
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
ament_target_dependencies(stereo_node rclcpp sensor_msgs geometry_msgs cv_bridge message_filters)
set_target_properties(stereo_node PROPERTIES INSTALL_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib;$ENV{HOME}/.cache/xtdrone/orbslam3/pangolin-install/lib")
# DT_RPATH, not DT_RUNPATH: it also applies to the libraries' own dependencies (libORB_SLAM3.so -> Pangolin)
target_link_options(stereo_node PRIVATE -Wl,--disable-new-dtags)
install(TARGETS stereo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
