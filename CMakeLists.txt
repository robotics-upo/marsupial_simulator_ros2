cmake_minimum_required(VERSION 3.5)
project(marsupial_simulator_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include 
  ${GAZEBO_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS}
)

# Build the plugins
add_library(plugin_ugv SHARED src/plugin_ugv.cpp)
ament_target_dependencies(plugin_ugv rclcpp geometry_msgs)
target_link_libraries(plugin_ugv ${GAZEBO_LIBRARIES})

add_library(tether_position_publisher SHARED src/tether_position_publisher.cpp)
ament_target_dependencies(tether_position_publisher rclcpp geometry_msgs gazebo_ros)
target_link_libraries(tether_position_publisher ${GAZEBO_LIBRARIES})

install(
  TARGETS 
  plugin_ugv
  tether_position_publisher
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    src
    scripts
    launch
    models
    urdf
    worlds
    simulation_data
    plots
    optimized_path
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/attach_tether.py
  scripts/jinja_gen.py
  scripts/ugv_control_keyboard.py
  scripts/ugv_control_controller.py
  scripts/spawn_ugv.py
  scripts/uav_to_point.py
  scripts/ugv_to_point.py
  scripts/ugv_theter_trajectory_follower.py
  scripts/plot_bag_data.py
  scripts/plot_bag_data_theatre.py
  scripts/get_bag_data.py
  scripts/get_bag_data_theatre.py
  scripts/trajectory_follower.py
  scripts/uav_trajectory_follower.py
  scripts/tether_comparison.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
