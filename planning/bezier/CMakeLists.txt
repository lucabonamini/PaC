project(bezier)
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

list(APPEND SRCS
  src/bezier.cpp
)

add_library(BezierPlanner SHARED src/bezier.cpp)
add_library(Planning::BezierPlanner ALIAS BezierPlanner)
target_include_directories(BezierPlanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(BezierPlanner
  PRIVATE
      Eigen3::Eigen
)
target_compile_options(BezierPlanner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
  set(CMAKE_CXX_CLANG_TIDY "")
  message("Compiling Bezier Planner tests")
  add_subdirectory(tests)
endif()

install(TARGETS BezierPlanner
    EXPORT BezierPlanner-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT BezierPlanner-export
    FILE BezierPlannerTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/BezierPlanner
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/frenet"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/BezierPlannerConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/frenet"
)
