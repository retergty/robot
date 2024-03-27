cmake_minimum_required(VERSION 3.20)

set(CAMKE_C_COMPILER gcc)
set(CAMKE_CXX_COMPILER g++)

project(ROBOT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TESTING "enable testing" OFF)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")

add_executable(robot LegsRobot.cpp LegsRobot.hpp Tree.hpp)

target_include_directories(robot PUBLIC eigen-3.4.0/Eigen)


if(TESTING)
  add_subdirectory(TEST)
endif()