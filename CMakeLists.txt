#################### BASIC SETUP ####################
cmake_minimum_required(VERSION 3.16)
project(EVALIO VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

#################### IMPORT DEPENDENCIES ####################
# Eigen 
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  include(FetchContent)
  message(STATUS "Pulling pybind11 from git...")
  FetchContent_Declare(
      pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      GIT_TAG        v2.13.6
  )
  FetchContent_MakeAvailable(pybind11)
endif()

#################### ADD TARGETS ####################
add_subdirectory(cpp)

pybind11_add_module(_cpp cpp/bindings.cpp)
target_link_libraries(_cpp PUBLIC pipelines Eigen3::Eigen)

install(TARGETS _cpp DESTINATION evalio)