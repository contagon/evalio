#################### BASIC SETUP ####################
cmake_minimum_required(VERSION 3.16)
project(EVALIO VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

#################### C++ Half ####################
# Eigen 
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(Eigen3)
add_subdirectory(cpp)

#################### Python Bindings ####################

option(EVALIO_BUILD_PYTHON "Build Python bindings" OFF)

if(EVALIO_BUILD_PYTHON)
    # Pull pybind11 if it's not available already
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
        FIND_PACKAGE_ARGS
    )
    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(_cpp cpp/bindings.cpp)

    target_link_libraries(_cpp PUBLIC evalio Eigen3::Eigen)
    install(TARGETS _cpp DESTINATION evalio)

    # TODO: This breaks import for some reason.. will have to investigate
    # Build stubs
    # set(STUBGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    # message(STATUS "STUBGEN_OUTPUT_DIR: ${STUBGEN_OUTPUT_DIR}")
    # add_custom_command(
    #     TARGET _cpp POST_BUILD
    #     COMMAND
    #       "${Python_EXECUTABLE}" -m pybind11_stubgen _cpp -o "${STUBGEN_OUTPUT_DIR}"
    #     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    #     COMMENT "Generating pybind11 stubs"
    #     VERBATIM
    # )
    # install(DIRECTORY ${STUBGEN_OUTPUT_DIR}/_cpp DESTINATION evalio)
endif()
