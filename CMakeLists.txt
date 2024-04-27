cmake_minimum_required(VERSION 3.20)

set(CAMKE_C_COMPILER gcc)
set(CAMKE_CXX_COMPILER g++)

project(ROBOT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TESTING "enable testing" OFF)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")
set(CMAKE_CXX_FLAGS "-m64 -Wa,-mbig-obj")
add_executable(simulation LegsRobot.cpp)
add_executable(lejurobot lejurobot.cpp)
add_executable(Calculateinitialcom CalculateInitialCom.cpp)
add_library(include_dir INTERFACE)

target_include_directories(include_dir INTERFACE eigen-3.4.0/Eigen Include)

target_link_libraries(simulation PRIVATE include_dir)
target_link_libraries(lejurobot PRIVATE include_dir)
target_link_libraries(Calculateinitialcom PRIVATE include_dir)

add_subdirectory(MatLab)

if(TESTING)
  enable_testing()
  add_subdirectory(TEST)
endif()