cmake_minimum_required(VERSION 3.10)
project(TCPClient)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(client client.cpp)
add_executable(get_calib_img get_calib_img.cpp)

target_link_libraries(client ${OpenCV_LIBS})
target_link_libraries(get_calib_img ${OpenCV_LIBS})