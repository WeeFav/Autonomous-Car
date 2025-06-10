cmake_minimum_required(VERSION 3.10)
project(TCPClient)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(client client.cpp)