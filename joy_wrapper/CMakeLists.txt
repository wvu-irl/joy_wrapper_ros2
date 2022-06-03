cmake_minimum_required(VERSION 3.5)
project(joy_wrapper)

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
find_package(builtin_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#find_package(rosidl_default_runtime REQUIRED)
find_package(joy_wrapper_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

include_directories(include)

add_library(joy_wrapper_lib
  ${PROJECT_SOURCE_DIR}/src/JoyWrapper.cpp
)

set(DEPENDENCIES
  "rclcpp"
  "joy_wrapper_msgs"
  "sensor_msgs"
)
ament_target_dependencies(joy_wrapper_lib ${DEPENDENCIES})


add_executable(joy_wrapper_node src/joy_wrapper_node.cpp)
target_link_libraries(joy_wrapper_node 
  joy_wrapper_lib 
  ${rclcpp_LIBRARIES}
)
# add_dependencies(epos_interface_node ${PROJECT_NAME}/src/epos_interface_node.cpp)
ament_target_dependencies(joy_wrapper_node 
  rclcpp 
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install (TARGETS
  joy_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
