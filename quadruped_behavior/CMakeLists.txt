cmake_minimum_required(VERSION 3.20)
project(quadruped_behavior)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Link Libraries:
add_executable(calibration_node src/calibrate.cpp)
target_link_libraries(calibration_node PUBLIC
  rclcpp::rclcpp
  dynamixel_sdk::dynamixel_sdk
  yaml-cpp::yaml-cpp
)

add_executable(static_position_node src/Static_Position.cpp)
target_link_libraries(static_position_node PUBLIC
  rclcpp::rclcpp
  dynamixel_sdk::dynamixel_sdk
  yaml-cpp::yaml-cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
)

install(TARGETS
  calibration_node
  static_position_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Set up Install Directories:
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
