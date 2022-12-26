cmake_minimum_required(VERSION 3.15)

project(dwa)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(Dwa SHARED src/dwa.cpp)
add_library(Control::Dwa ALIAS Dwa)
target_include_directories(Dwa PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(Dwa
    PRIVATE
      Utilities::Utilities
      Models::Unicycle
)
target_compile_options(Dwa PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if(USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling Dwa tests")
    add_subdirectory(tests)
endif()

# install(TARGETS Dwa
#     EXPORT Dwa-export
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
# )

# install(EXPORT Dwa-export
#     FILE DwaTargets.cmake
#     NAMESPACE Control::
#     DESTINATION lib/cmake/Dwa
# )
# install(
#     DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
#     DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/dwa"
# )
# install(
#     FILES
#         ${CMAKE_CURRENT_SOURCE_DIR}/cmake/DwaConfig.cmake
#     DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dwa"
# )