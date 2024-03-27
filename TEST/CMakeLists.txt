enable_testing()
add_subdirectory(googletest)

add_executable(kinematics kinematics_test.cpp)

set(test_targets
    kinematics)

foreach(test_target IN LISTS test_targets)
  get_property(target_srcs TARGET ${test_target} PROPERTY SOURCES)
  foreach(target_src IN LISTS target_srcs)
  add_custom_command(OUTPUT ./CMakeFiles/${test_target}.dir/clear_${test_target}_${target_src}_coverage.h
    DEPENDS ${target_src}
    COMMAND ${CMAKE_COMMAND} -E echo "clearing ${target_src}.gcno"
    COMMAND ${CMAKE_COMMAND} -E rm -f ./${target_src}.gcno
    COMMAND ${CMAKE_COMMAND} -E touch clear_${test_target}_${target_src}_coverage.h
    WORKING_DIRECTORY ./CMakeFiles/${test_target}.dir
    )
    target_sources(${test_target} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${test_target}.dir/clear_${test_target}_${target_src}_coverage.h")
  
    list(APPEND ${test_target}_gcda ./${target_src}.gcda)
  endforeach()
  
  add_custom_target(clear_${test_target}_gcda 
  COMMAND ${CMAKE_COMMAND} -E echo "clearing ${${test_target}_gcda}"
  COMMAND ${CMAKE_COMMAND} -E rm -f "${${test_target}_gcda}" 
  WORKING_DIRECTORY ./CMakeFiles/${test_target}.dir
  COMMAND_EXPAND_LISTS)
    
  add_dependencies(${test_target} clear_${test_target}_gcda)
endforeach()

foreach(test_target IN LISTS test_targets)
  target_link_libraries(${test_target} PRIVATE gtest gmock gtest_main gmock_main)
  target_compile_options( ${test_target} PRIVATE
    -pedantic
    -Wall
    -Warray-bounds
    -Wcast-align
    -Wcast-qual
    -Wdisabled-optimization
    -Wextra
    -Wfloat-equal
    -Wformat-security
    -Wformat=2
    -Winit-self
    -Wlogical-op
    -Wmissing-declarations
    -Wmissing-include-dirs
    -Wno-sign-compare
    -Wno-unused
    -Wno-unused-parameter
    -Wnoexcept
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpointer-arith
    -Wredundant-decls
    -Wreorder
    -Wshadow
    -Wsign-conversion
    -Wsign-promo
    -Wstrict-null-sentinel
    -Wswitch-default
    -Wuninitialized
    -Wunused-variable
    -Wstrict-overflow=4
    )
    if(CMAKE_SYSTEM_NAME MATCHES "Windows.*")
      message("System is windows,doesn't support coverage test")
    elseif(CMAKE_SYSTEM_NAME MATCHES "CYGWIN.*")
      target_compile_options(${test_target} PRIVATE --coverage)
      message("System is cygwin")
    endif()
endforeach()