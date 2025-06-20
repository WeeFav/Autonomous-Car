cmake_minimum_required(VERSION 3.10)
project(Jetson)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(server server.cpp)
add_executable(spi spi.cpp)
add_executable(spidev_test spidev_test.c)

target_link_libraries(server ${OpenCV_LIBS})