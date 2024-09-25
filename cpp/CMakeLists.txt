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
pybind11_add_module(_pipelines bindings.cpp)
target_link_libraries(_pipelines PUBLIC pipelines gtsam)

install(TARGETS _pipelines DESTINATION evalio/pipelines)