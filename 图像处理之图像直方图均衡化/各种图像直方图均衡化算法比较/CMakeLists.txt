cmake_minimum_required(VERSION 2.8)

project(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV 3 REQUIRED)

message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})

add_executable(main main.cpp)

target_link_libraries(main ${OpenCV_LIBS})
