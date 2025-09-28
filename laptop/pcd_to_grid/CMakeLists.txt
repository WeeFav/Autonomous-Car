cmake_minimum_required(VERSION 3.10)
project(pcd_to_grid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL REQUIRED COMPONENTS io common filters visualization)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pcd_to_grid src/pcd_to_grid.cpp)

target_link_libraries(pcd_to_grid
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
