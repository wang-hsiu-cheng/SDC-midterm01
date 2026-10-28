cmake_minimum_required(VERSION 3.8)
project(localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(localization_core
  src/gnss_prior.cpp
  src/ground_height_prior.cpp
  src/imu_processor.cpp
  src/initial_pose_search.cpp
  src/map_manager.cpp
  src/measurement_fusion.cpp
  src/ndt_matcher.cpp
  src/road_heading_prior.cpp
  src/side_balance.cpp
  src/state_estimator.cpp
  src/trail_heading.cpp
)
ament_target_dependencies(localization_core
  rclcpp
  sensor_msgs
  geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
)
target_link_libraries(localization_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(matcher_bench src/matcher_bench.cpp)
target_link_libraries(matcher_bench ${PCL_LIBRARIES} Eigen3::Eigen)
install(TARGETS matcher_bench DESTINATION lib/${PROJECT_NAME})

add_executable(localization src/localization.cpp)
ament_target_dependencies(localization
  rosbag2_cpp
  rosbag2_storage
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)
target_link_libraries(localization
  localization_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
)

add_executable(map_modified src/map_modified.cpp)
ament_target_dependencies(map_modified
  rclcpp
)
target_link_libraries(map_modified
  ${PCL_LIBRARIES}
)

add_executable(visualization src/visualization.cpp)
ament_target_dependencies(visualization
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)
target_link_libraries(visualization
  ${PCL_LIBRARIES}
)

install(
  TARGETS
    localization
    map_modified
    visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
