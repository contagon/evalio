# Create Library
add_library(evalio INTERFACE)

# Set Library Properties
target_include_directories(evalio INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/..")

# TODO: Add in install method to include in python package
