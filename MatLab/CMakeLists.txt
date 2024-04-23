if(CMAKE_SYSTEM_NAME MATCHES "Windows.*")
  configure_file(preview_control_parameters_generate.txt prvctl_parm.hpp NEWLINE_STYLE WIN32)
  configure_file(preview_test1.txt preview_test1.hpp NEWLINE_STYLE WIN32)
  configure_file(preview_test2.txt preview_test2.hpp NEWLINE_STYLE WIN32)
elseif(CMAKE_SYSTEM_NAME MATCHES "CYGWIN.*")
  configure_file(preview_control_parameters_generate.txt prvctl_parm.hpp NEWLINE_STYLE WIN32)
  configure_file(preview_test1.txt preview_test1.hpp NEWLINE_STYLE WIN32)
  configure_file(preview_test2.txt preview_test2.hpp NEWLINE_STYLE WIN32)
elseif(CMAKE_SYSTEM_NAME MATCHES "CYGWIN.*")
  configure_file(preview_control_parameters_generate.txt prvctl_parm.hpp NEWLINE_STYLE UNIX)
  configure_file(preview_test1.txt preview_test1.hpp NEWLINE_STYLE UNIX)
  configure_file(preview_test2.txt preview_test2.hpp NEWLINE_STYLE UNIX)
endif()

set(should_run_matlab_script
  preview_control_parameters_generate
  preview_test1
  preview_test2)

foreach(m_src IN LISTS should_run_matlab_script)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/${m_src}.txt"
    COMMAND matlab -nosplash -nodesktop -nodisplay -noFigureWindows -r "run('${m_src}.m'); exit;"
    COMMAND ${CMAKE_COMMAND} -E echo "generate ${m_src}.txt now"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS ${m_src}.m)
  add_custom_target(gen_${m_src}_txt ALL
    SOURCES ${m_src}.txt
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endforeach()

target_include_directories(include_dir INTERFACE "${CMAKE_CURRENT_BINARY_DIR}")