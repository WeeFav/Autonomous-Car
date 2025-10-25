cmake_minimum_required(VERSION 3.8)
project(cloud_saver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)

add_executable(cloud_saver src/cloud_saver.cpp)

ament_target_dependencies(cloud_saver
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(cloud_saver
  ${PCL_LIBRARIES}
)

install(TARGETS
  cloud_saver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
