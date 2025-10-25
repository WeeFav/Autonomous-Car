cmake_minimum_required(VERSION 3.8)
project(camera_tcp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_publisher ${OpenCV_LIBRARIES})

add_executable(camera_publisher_tcp src/camera_publisher_tcp.cpp)
ament_target_dependencies(camera_publisher_tcp rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_publisher_tcp ${OpenCV_LIBRARIES})

install(TARGETS
  camera_publisher
  camera_publisher_tcp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
