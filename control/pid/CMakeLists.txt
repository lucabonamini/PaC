cmake_minimum_required(VERSION 3.15)

project(pid)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(Pid SHARED src/pid.cpp)
add_library(Control::Pid ALIAS Pid)
target_include_directories(Pid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(Pid
    PRIVATE
        Utilities::Utilities
)
target_compile_options(Pid PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling Pid tests")
    add_subdirectory(tests)
endif()

install(TARGETS Pid
    EXPORT Pid-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT Pid-export
    FILE PidTargets.cmake
    NAMESPACE Control::
    DESTINATION lib/cmake/Pid
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/pid"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Pid"
)