cmake_minimum_required(VERSION 3.5)
project(ros_assignments)

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
find_package(Curses REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(ros_assignments "srv/UserCommand.srv" "srv/PositionGenerator.srv")

add_executable(position_generation_server src/position_generation_service.cpp)
ament_target_dependencies(position_generation_server rclcpp)
rosidl_target_interfaces(position_generation_server ros_assignments "rosidl_typesupport_cpp")
install(TARGETS position_generation_server DESTINATION lib/ros_assignments)

add_executable(user_interface src/user_interface.cpp)
target_link_libraries(user_interface ${CURSES_LIBRARIES})
ament_target_dependencies(user_interface rclcpp)
rosidl_target_interfaces(user_interface ros_assignments "rosidl_typesupport_cpp")
install(TARGETS user_interface DESTINATION lib/ros_assignments)

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

ament_package()

install(FILES mapping_rules.yaml DESTINATION share/ros_assignments)
