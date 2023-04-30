include(CMakeParseArguments)
include(ProcessorCount)

# Determine the number of cores available on this system
set(PROCESSOR_COUNT "1")
ProcessorCount(N)
if(NOT N EQUAL 0)
  set(PROCESSOR_COUNT ${N})
endif()
message(STATUS "Processor Count : ${PROCESSOR_COUNT}")

## find clang tool binaries on the system
# clang-format
set(CLANG_FORMAT_PROG_NAME "clang-format")
find_program(CLANG_FORMAT_PROG ${CLANG_FORMAT_PROG_NAME} DOC "'clang-format' executable")
if(CLANG_FORMAT_PROG AND NOT TARGET format)
  add_custom_target(format)
else()
  message(WARNING "Could not locate ${CLANG_FORMAT_PROG_NAME} on the system.")
endif()
# clang-tidy
set(CLANG_TIDY_PROG_NAME "run-clang-tidy-10.py")
find_program(CLANG_TIDY_PROG ${CLANG_TIDY_PROG_NAME} DOC "'clang-tidy' executable")
if(CLANG_TIDY_PROG AND NOT TARGET tidy)
  if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    message(WARNING "Invoke Catkin/CMake with '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
                     to generate compilation database for 'clang-tidy'.")
  endif()

  add_custom_target(tidy)
else()
  message(WARNING "Could not locate ${CLANG_TIDY_PROG_NAME} on the system.")
endif()

function(add_format_target _target)
  if(NOT CLANG_FORMAT_PROG)
    return()
  endif()
  cmake_parse_arguments(ARG "" "" "FILES" ${ARGN})

  add_custom_target(format-${_target}
    COMMAND ${CLANG_FORMAT_PROG} -i ${ARG_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMENT "Formatting ${_target} source code with `${CLANG_FORMAT_PROG_NAME}`"
    VERBATIM
  )
  add_dependencies(format format-${_target})
endfunction()

function(add_tidy_target _target)
  if(NOT CLANG_TIDY_PROG)
    return()
  endif()
  cmake_parse_arguments(ARG "" "" "FILES;DEPENDS" ${ARGN})

  add_custom_target(tidy-${_target}
    COMMAND ${CLANG_TIDY_PROG} -fix -format -style "file" -p=${CMAKE_BINARY_DIR} -j ${PROCESSOR_COUNT} -quiet ${ARG_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    DEPENDS ${ARG_DEPENDS}
    COMMENT "Running `${CLANG_TIDY_PROG_NAME}` for ${_target}"
    VERBATIM
  )
  add_dependencies(tidy tidy-${_target})
endfunction()
