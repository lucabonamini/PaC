project(frenet)
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

add_library(FrenetPlanner SHARED src/frenet.cpp)
add_library(Planning::FrenetPlanner ALIAS FrenetPlanner)
target_include_directories(FrenetPlanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(FrenetPlanner
    PRIVATE
        Eigen3::Eigen
        Utilities::Utilities
        Planning::CubicSplinePlanner
        Planning::QuarticPolynomialPlanner
        Planning::QuinticPolynomialPlanner
        glog
)
target_compile_options(FrenetPlanner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
  set(CMAKE_CXX_CLANG_TIDY "")
  message("Compiling Frenet Planner tests")
  add_subdirectory(tests)
endif()

install(TARGETS FrenetPlanner
    EXPORT FrenetPlanner-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT FrenetPlanner-export
    FILE FrenetPlannerTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/FrenetPlanner
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/frenet"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FrenetPlannerConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/frenet"
)
