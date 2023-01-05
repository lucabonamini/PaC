cmake_minimum_required(VERSION 3.15)

project(quintic_polynomial_planner)

set(CMAKE_CXX_CLANG_TIDY "")

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

add_library(QuinticPolynomialPlanner SHARED src/quintic_polynomial_planner.cpp)
add_library(Planning::QuinticPolynomialPlanner ALIAS QuinticPolynomialPlanner)
target_include_directories(QuinticPolynomialPlanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(QuinticPolynomialPlanner
    PRIVATE
        Eigen3::Eigen
)
target_compile_options(QuinticPolynomialPlanner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling Quartic Polynomial Planner tests")
    add_subdirectory(tests)
endif()

install(TARGETS QuinticPolynomialPlanner
    EXPORT QuinticPolynomialPlanner-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT QuinticPolynomialPlanner-export
    FILE QuinticPolynomialPlannerTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/QuinticPolynomialPlanner
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/quintic_polynomial_planner"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/QuinticPolynomialPlannerConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/quintic_polynomial_planner"
)