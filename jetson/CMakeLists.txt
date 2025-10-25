cmake_minimum_required(VERSION 3.22)
project(Jetson)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v6.1.9
)
FetchContent_MakeAvailable(ftxui)

add_executable(server server.cpp)
add_executable(uart uart.cpp)
add_executable(calibrate_camera calibrate_camera.cpp)

target_link_libraries(server ${OpenCV_LIBS})

target_link_libraries(calibrate_camera ${OpenCV_LIBS} stdc++fs)

target_link_libraries(uart PRIVATE
    pthread
    ftxui::component
    ftxui::dom
    ftxui::screen
)