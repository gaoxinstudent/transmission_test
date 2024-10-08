cmake_minimum_required(VERSION 3.8)
project(key_feedback)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
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

add_executable(keyboard_publisher src/keyboard_publisher.cpp)
target_link_libraries(keyboard_publisher ${CURSES_LIBRARIES})
ament_target_dependencies(keyboard_publisher rclcpp std_msgs)

add_executable(keyboard_subscriber src/keyboard_subscriber.cpp)
ament_target_dependencies(keyboard_subscriber rclcpp std_msgs)

install(TARGETS keyboard_publisher
      keyboard_subscriber
    DESTINATION lib/${PROJECT_NAME})

ament_package()
