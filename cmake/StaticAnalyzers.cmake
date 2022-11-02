option(ENABLE_INCLUDE_WHAT_YOU_USE "Enable static analysis with include-what-you-use" ON)
option(ENABLE_CPPCHECK "Enable static analysis with cppcheck" ON)
option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" ON)
option(ENABLE_CLANG_FORMAT "Enable automatic formatting with clang-format" ON)

if(ENABLE_INCLUDE_WHAT_YOU_USE)
  find_program(INCLUDE_WHAT_YOU_USE include-what-you-use)
  if(INCLUDE_WHAT_YOU_USE)
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${INCLUDE_WHAT_YOU_USE})
  else()
    message(SEND_ERROR "include-what-you-use requested but executable not found")
  endif()
endif()

if(ENABLE_CPPCHECK)
  find_program(CPPCHECK cppcheck)
  if(CPPCHECK)
    set(CMAKE_CXX_CPPCHECK
        ${CPPCHECK}
        --suppress=missingInclude
        --enable=style
        --inline-suppr
        --inconclusive)
    if(WARNINGS_AS_ERRORS)
      list(APPEND CMAKE_CXX_CPPCHECK --error-exitcode=2)
    endif()
  else()
    message(SEND_ERROR "cppcheck requested but executable not found")
  endif()
endif()

if(ENABLE_CLANG_TIDY)
  find_program(CLANGTIDY clang-tidy)
  if(CLANGTIDY)
    set(CMAKE_CXX_CLANG_TIDY
      ${CLANGTIDY}
      -extra-arg=-Wno-unknown-warning-option)
    if(WARNINGS_AS_ERRORS)
      list(APPEND CMAKE_CXX_CLANG_TIDY -warnings-as-errors=*)
    endif()
  else()
    message(SEND_ERROR "clang-tidy requested but executable not found")
  endif()
endif()

if(ENABLE_CLANG_FORMAT)
  set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(CHECK_DIRS utilities models planning control)
  foreach(DIR ${CHECK_DIRS})
    message("Running clang-format on ${DIR} folder")
    execute_process(COMMAND python3 ${MAIN_DIR}/run-clang-format.py -r ${MAIN_DIR}/${DIR} -i)
  endforeach()
endif()
