cmake_minimum_required(VERSION 3.20)

set(CAMKE_C_COMPILER gcc)
set(CAMKE_CXX_COMPILER g++)

project(ROBOT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(robot LegsRobot.cpp LegsRobot.hpp)

target_include_directories(robot PUBLIC eigen-3.4.0/Eigen)

