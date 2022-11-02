cmake_minimum_required(VERSION 3.15)

project(Utilities)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(Utilities SHARED src/math.cpp src/types.cpp)
add_library(Utilities::Utilities ALIAS Utilities)
target_include_directories(Utilities PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(Utilities PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

install(TARGETS Utilities
    EXPORT Utilities-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT Utilities-export
    FILE UtilitiesTargets.cmake
    NAMESPACE Utilities::
    DESTINATION lib/cmake/Utilities
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/utilities"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/UtilitiesConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Utilities"
)
