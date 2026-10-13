cmake_minimum_required(VERSION 3.16)
project(xtdrone_ugv_planning)

# ROS 2 port of XTDrone's motion_planning/ugv_planning (chapter 31): hybrid_astar_test (hybrid A* path
# search with a Voronoi-based smoother) and pure_pursuit (path follower) for the CAT Vehicle UGV.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED)

add_executable(hybrid_astar_node
  src/apps/main.cpp
  src/navigation/path_search.cpp
  src/subscriber/start_subscriber.cpp
  src/subscriber/est_start_subscriber.cpp
  src/subscriber/goal_subscriber.cpp
  src/subscriber/costmap_subscriber.cpp
  src/algorithm/path_searcher/hybrid_a_star/collision_detection.cpp
  src/algorithm/path_searcher/hybrid_a_star/dubins.cpp
  src/algorithm/path_searcher/hybrid_a_star/node3d.cpp
  src/algorithm/path_searcher/hybrid_a_star/node2d.cpp
  src/global_definition/constants.cpp
  src/algorithm/path_searcher/hybrid_a_star/algorithm.cpp
  src/algorithm/path_searcher/hybrid_a_star/visualize.cpp
  src/algorithm/path_searcher/hybrid_a_star/path.cpp
  src/algorithm/path_searcher/hybrid_a_star/bucketedqueue.cpp
  src/algorithm/path_searcher/hybrid_a_star/dynamic_voronoi.cpp
  src/algorithm/path_searcher/hybrid_a_star/smoother.cpp
  src/publisher/path_publisher.cpp
  src/publisher/tf_broadcaster.cpp
  src/publisher/points_publisher.cpp)
target_include_directories(hybrid_astar_node PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
target_link_libraries(hybrid_astar_node yaml-cpp ${OMPL_LIBRARIES})
ament_target_dependencies(hybrid_astar_node rclcpp ament_index_cpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(pure_pursuit_node src/pure_pursuit/pure_pursuit.cpp)
target_include_directories(pure_pursuit_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(pure_pursuit_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS hybrid_astar_node pure_pursuit_node odom_to_tf DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
ament_package()
