cmake_minimum_required(VERSION 3.5)
project(day1)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(day1 main.cpp)
target_link_libraries(day1 ${OpenCV_LIBS})