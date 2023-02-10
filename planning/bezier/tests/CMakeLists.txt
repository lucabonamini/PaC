set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

add_executable(example_bezier example_bezier.cpp)
target_link_libraries(example_bezier
  ${OpenCV_LIBS}
  Planning::BezierPlanner
)
target_compile_options(example_bezier PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
