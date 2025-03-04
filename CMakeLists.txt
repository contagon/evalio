#################### BASIC SETUP ####################
cmake_minimum_required(VERSION 3.16)
project(EVALIO VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#################### IMPORT DEPENDENCIES ####################
# Eigen 
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(pybind11)

#################### ADD TARGETS ####################
add_subdirectory(cpp)

# Build python bindings
pybind11_add_module(_cpp cpp/bindings.cpp)
target_link_libraries(_cpp PUBLIC evalio_cpp Eigen3::Eigen)
install(TARGETS _cpp DESTINATION evalio)

# Build stubs
# set(STUBGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
# add_custom_command(
#     TARGET _cpp POST_BUILD
#     COMMAND
#       "${Python_EXECUTABLE}" -m pybind11_stubgen _cpp -o "${STUBGEN_OUTPUT_DIR}"
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#     COMMENT "Generating pybind11 stubs"
#     VERBATIM
# )
# install(DIRECTORY ${STUBGEN_OUTPUT_DIR}/_cpp DESTINATION evalio)