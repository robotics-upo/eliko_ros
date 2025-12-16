cmake_minimum_required(VERSION 3.8)
project(eliko_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(distance_monitor src/distance_monitor.cpp)
ament_target_dependencies(distance_monitor rclcpp std_msgs)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  distance_monitor
  DESTINATION lib/${PROJECT_NAME})

ament_package()