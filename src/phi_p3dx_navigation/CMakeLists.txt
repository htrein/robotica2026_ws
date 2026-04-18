cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)


include_directories(include)

# ---------- C++ executable node ----------
add_executable(control_example_cpp src/control_example.cpp src/navigation_node.cpp)
ament_target_dependencies(control_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(wall_follower_pid src/wall_follower_pid.cpp src/navigation_node.cpp)
ament_target_dependencies(wall_follower_pid
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(vfh_simplified src/vfh_simplified.cpp src/navigation_node.cpp)
ament_target_dependencies(vfh_simplified
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS control_example_cpp wall_follower_pid vfh_simplified
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config scripts
        DESTINATION share/${PROJECT_NAME})

# ---------- Python Node script ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/control_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_example_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
