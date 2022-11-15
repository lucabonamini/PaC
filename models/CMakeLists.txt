cmake_minimum_required(VERSION 3.15)

project(Models)

add_library(Models INTERFACE)
add_library(Models::Models ALIAS Models)
target_include_directories(Models INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(Models INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)


add_library(Unicycle SHARED src/unicycle.cpp)
add_library(Models::Unicycle ALIAS Unicycle)
target_include_directories(Unicycle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(Unicycle
  PUBLIC
    Utilities::Utilities
)
target_compile_options(Unicycle PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

add_library(DynamicUnicycle SHARED src/dynamic_unicycle.cpp)
add_library(Models::DynamicUnicycle ALIAS DynamicUnicycle)
target_include_directories(Unicycle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(DynamicUnicycle
  PUBLIC
    Utilities::Utilities
)
target_compile_options(DynamicUnicycle PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

install(TARGETS Models
    EXPORT Models-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT Models-export
    FILE ModelsTargets.cmake
    NAMESPACE Models::
    DESTINATION lib/cmake/Models
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/models"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Models"
)