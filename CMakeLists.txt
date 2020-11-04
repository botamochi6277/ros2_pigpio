cmake_minimum_required(VERSION 3.5)
project(ros2_pigpio)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add include directories
include_directories(include ./include)

# gpio_reader
add_executable(gpio_reader src/gpio_reader.cpp)
target_link_libraries(gpio_reader pigpiod_if2)
ament_target_dependencies(gpio_reader rclcpp std_msgs)

# gpio_writer
add_executable(gpio_writer src/gpio_writer.cpp)
target_link_libraries(gpio_writer pigpiod_if2)
ament_target_dependencies(gpio_writer rclcpp std_msgs)

# gpio_pwm_writer
add_executable(gpio_pwm_writer src/gpio_pwm_writer.cpp)
target_link_libraries(gpio_pwm_writer pigpiod_if2)
ament_target_dependencies(gpio_pwm_writer rclcpp std_msgs)

# wheels
add_executable(wheels src/wheels.cpp)
target_link_libraries(wheels pigpiod_if2)
ament_target_dependencies(wheels rclcpp std_msgs geometry_msgs)

# subclass test
add_executable(subclass src/subclass.cpp)
target_link_libraries(subclass pigpiod_if2)
ament_target_dependencies(subclass rclcpp std_msgs)


# Install built nodes
install(TARGETS
  gpio_reader
  gpio_writer
  gpio_pwm_writer
  wheels
  subclass
  DESTINATION lib/${PROJECT_NAME})


ament_package()
