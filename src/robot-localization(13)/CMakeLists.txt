cmake_minimum_required(VERSION 3.8)
project(robot_localization)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Create all executables (now we have all the source files)
add_executable(raspberry_localization_node src/raspberry_localization_node.cpp)
add_executable(localization_node src/localization_node.cpp)
add_executable(standard_localization_node src/standard_localization_node.cpp)
add_executable(odom_imu_fusion_node src/odom_imu_fusion_node.cpp)
add_executable(odom_tf_fusion_node src/odom_tf_fusion_node.cpp)

# List of all executables
set(EXECUTABLES
  raspberry_localization_node
  localization_node
  standard_localization_node
  odom_imu_fusion_node
  odom_tf_fusion_node
)

# Configure each executable
foreach(EXECUTABLE ${EXECUTABLES})
  target_include_directories(${EXECUTABLE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  )

  # Link dependencies for all executables
  ament_target_dependencies(${EXECUTABLE}
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    tf2_msgs
  )
endforeach()

# Install all executables
install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (if any)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
  )
endif()

ament_package()
