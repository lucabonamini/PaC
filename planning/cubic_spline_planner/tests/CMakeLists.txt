set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_link_libraries(example_cubic_spline_planner
  Utilities::Utilities
  Planning::CubicSplinePlanner
  ${OpenCV_LIBS}
)
target_compile_options(example_cubic_spline_planner PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)