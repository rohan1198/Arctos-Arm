cmake_minimum_required(VERSION 3.8)
project(arctos_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# moveit_convert executable
add_executable(moveit_convert src/moveit_convert.cpp)
ament_target_dependencies(moveit_convert
  rclcpp
  std_msgs
  sensor_msgs
)

# move_group_interface executable  
add_executable(move_group_interface src/move_group_interface_coor_1.cpp)
ament_target_dependencies(move_group_interface
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link interface libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_convert "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  moveit_convert
  move_group_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/interface.py
  scripts/transform.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
