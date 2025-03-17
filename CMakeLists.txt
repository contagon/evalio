# ------------------------- Basics ------------------------- #
cmake_minimum_required(VERSION 3.16)
project(EVALIO VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

# ------------------------- Dependencies ------------------------- #
# Pull eigen if it's not available already
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    FIND_PACKAGE_ARGS NO_MODULE # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(Eigen3)

# ------------------------- Main Header Library ------------------------- #
add_subdirectory(cpp/evalio)

# ------------------------- Python Bindings ------------------------- #
option(EVALIO_BUILD_PYTHON "Build Python bindings" OFF)
if(EVALIO_BUILD_PYTHON)
    add_subdirectory(cpp/bindings)
endif()
