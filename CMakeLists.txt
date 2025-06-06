# First thing, we check if vcpkg is installed and if so, use it for dependencies
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake"
    )
endif()

# ------------------------- Basics ------------------------- #
cmake_minimum_required(VERSION 3.24)
project(EVALIO VERSION 0.3.0 LANGUAGES CXX)

# Enforce C++17 for std::variant amongst others
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

# Make build a release build unless otherwise specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(
        CACHE CMAKE_BUILD_TYPE
        PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
    )
endif()

# ------------------------- Dependencies ------------------------- #
# Pull eigen if it's not available already
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    FIND_PACKAGE_ARGS
        NO_MODULE # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(Eigen3)

# ------------------------- Main Header Library ------------------------- #
add_subdirectory(cpp/evalio)

# ------------------------- Python Bindings ------------------------- #
option(EVALIO_BUILD_PYTHON "Build Python bindings" OFF)
if(EVALIO_BUILD_PYTHON)
    add_subdirectory(cpp/bindings)
endif()
