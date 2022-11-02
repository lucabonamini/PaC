cmake_minimum_required(VERSION 3.15)

project(adaptive_pure_pursuit)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(AdaptivePurePursuit SHARED src/adaptive_pure_pursuit.cpp)
add_library(Control::AdaptivePurePursuit ALIAS AdaptivePurePursuit)
target_include_directories(AdaptivePurePursuit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>    
)
target_link_libraries(AdaptivePurePursuit
    PRIVATE
        Utilities::Utilities
        Control::PurePursuit
)
target_compile_options(AdaptivePurePursuit PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if (USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling AdaptivePurePursuit tests")
    add_subdirectory(tests)
endif()

install(TARGETS AdaptivePurePursuit
    EXPORT AdaptivePurePursuit-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT AdaptivePurePursuit-export
    FILE AdaptivePurePursuitTargets.cmake
    NAMESPACE Planning::
    DESTINATION lib/cmake/AdaptivePurePursuit
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/adaptive_pure_pursuit"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/AdaptivePurePursuit"
)