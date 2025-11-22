cmake_minimum_required(VERSION 3.8)
project(orbslam_nav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch urdf rviz worlds models maps photos
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(vo src/vo.cpp)
ament_target_dependencies(vo rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(vo ${OpenCV_LIBS})

install(TARGETS
  vo
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()