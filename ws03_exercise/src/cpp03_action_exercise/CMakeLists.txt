cmake_minimum_required(VERSION 3.8)
project(cpp03_action_exercise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(exer04_action_server src/exer04_action_server.cpp)
add_executable(exer05_action_client src/exer05_action_client.cpp)

target_include_directories(exer04_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(exer04_action_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  exer04_action_server
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "turtlesim"
  "base_interfaces_demo"
)

ament_target_dependencies(
  exer05_action_client
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "turtlesim"
  "base_interfaces_demo"
)

install(TARGETS 
  exer04_action_server
  exer05_action_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_package()
