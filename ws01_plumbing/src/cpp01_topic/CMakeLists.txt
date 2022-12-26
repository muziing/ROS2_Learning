cmake_minimum_required(VERSION 3.8)
project(cpp01_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)

add_executable(demo01_talker_str src/demo01_talker_str.cpp)
add_executable(demo02_listener_str src/demo02_listener_str.cpp)
add_executable(demo03_talker_student src/demo03_talker_student.cpp)
add_executable(demo04_listener_student src/demo04_listener_student.cpp)

target_include_directories(demo01_talker_str PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(demo01_talker_str PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(demo01_talker_str
  "rclcpp"
  "std_msgs"
)

ament_target_dependencies(demo02_listener_str
  "rclcpp"
  "std_msgs"
)

ament_target_dependencies(demo03_talker_student
  "rclcpp"
  "base_interfaces_demo"
)

ament_target_dependencies(demo04_listener_student
  "rclcpp"
  "base_interfaces_demo"
)

install(TARGETS
  demo01_talker_str
  demo02_listener_str
  demo03_talker_student
  demo04_listener_student
  DESTINATION lib/${PROJECT_NAME})

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
