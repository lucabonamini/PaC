cmake_minimum_required(VERSION 3.15)

project(cubic_spline_planner)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

add_library(CubicSplinePlanner SHARED src/cubic_spline_planner.cpp)
add_library(Planning::CubicSplinePlanner ALIAS CubicSplinePlanner)
target_include_directories(CubicSplinePlanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(CubicSplinePlanner
  PUBLIC
    Utilities::Utilities
    Eigen3::Eigen # this should be PUBLIC since its header is in cubic_spline_planner.h
)
target_compile_options(CubicSplinePlanner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling CubicSplinePlanner tests")
    add_subdirectory(tests)
endif()

install(TARGETS CubicSplinePlanner
    EXPORT CubicSplinePlanner-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT CubicSplinePlanner-export
    FILE CubicSplinePlannerTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/CubicSplinePlanner
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/cubic_spline_planner"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CubicSplinePlannerConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CubicSplinePlanner"
)