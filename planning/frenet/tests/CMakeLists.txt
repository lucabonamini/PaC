set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

add_executable(example_frenet example_frenet.cpp)
target_link_libraries(example_frenet
  Utilities::Utilities
  Planning::CubicSplinePlanner
  Planning::QuarticPolynomialPlanner
  Planning::QuinticPolynomialPlanner
  Planning::FrenetPlanner
  ${OpenCV_LIBS}
  glog
)
target_compile_options(example_frenet PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
