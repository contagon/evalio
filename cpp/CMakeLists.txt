# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(SRCS "")
set(LIBS gtsam)
set(LIB_NAME pipelines)

# ------------------------- Import all pipelines ------------------------- #
# KISS-ICP
option(EVALIO_KISS_ICP "Build KISS-ICP" ON)
if(EVALIO_KISS_ICP)
    add_subdirectory(pipelines/kiss-icp/cpp/kiss_icp)
    list(APPEND LIBS kiss_icp_core)
endif()

# TODO: Figure out python bindings with these

# ------------------------- Put all wrappers into a target ------------------------- #
# find all source and header files

# Create Library
add_library(${LIB_NAME} "${SRCS}") 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${LIB_NAME} PUBLIC ${LIBS})
set_target_properties(${LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# ------------------------- Make python wrapper for pipelines ------------------------- #
# Add python bindings
# TODO: Fix saving directory
pybind11_add_module(evalio_python bindings.cpp)
target_link_libraries(evalio_python PUBLIC pipelines gtsam)

# set_target_properties(evalio_python PROPERTIES
#   INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
#   INSTALL_RPATH_USE_LINK_PATH TRUE
#   OUTPUT_NAME "evalio_python"
#   LIBRARY_OUTPUT_DIRECTORY "${PYTHON_BUILD_DIRECTORY}/evalio"
#   DEBUG_POSTFIX "" # Otherwise you will have a wrong name
#   RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
# )
# target_compile_features(evalio_python PUBLIC cxx_std_17)