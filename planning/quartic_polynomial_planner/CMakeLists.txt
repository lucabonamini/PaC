cmake_minimum_required(VERSION 3.15)

project(quartic_polynomial_planner)

set(CMAKE_CXX_CLANG_TIDY "")

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

add_library(QuarticPolynomialPlanner SHARED src/quartic_polynomial_planner.cpp)
add_library(Planning::QuarticPolynomialPlanner ALIAS QuarticPolynomialPlanner)
target_include_directories(QuarticPolynomialPlanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(QuarticPolynomialPlanner
    PRIVATE
        Eigen3::Eigen
)
target_compile_options(QuarticPolynomialPlanner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

install(TARGETS QuarticPolynomialPlanner
    EXPORT QuarticPolynomialPlanner-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT QuarticPolynomialPlanner-export
    FILE QuarticPolynomialPlannerTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/QuarticPolynomialPlanner
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/quartic_polynomial_planner"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/QuarticPolynomialPlannerConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/quartic_polynomial_planner"
)