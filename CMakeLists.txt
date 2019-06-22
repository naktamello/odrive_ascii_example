cmake_minimum_required(VERSION 3.2)
project(odrive_ascii)

set(CMAKE_CXX_STANDARD 14)

add_executable(odrive_ascii main.cpp)
TARGET_LINK_LIBRARIES(odrive_ascii boost_system boost_thread)