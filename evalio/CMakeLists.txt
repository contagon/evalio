cmake_minimum_required(VERSION 3.16)

# Create Library
add_library(evalio INTERFACE)

target_include_directories(evalio INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(evalio INTERFACE GTSAM)