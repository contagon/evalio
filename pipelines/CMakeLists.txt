# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(SRCS "")
set(LIBS "")
set(LIB_NAME pipelines)

# ------------------------- Import all pipelines ------------------------- #
# KISS-ICP
option(EVALIO_KISS_ICP "Build KISS-ICP" ON)
if(EVALIO_KISS_ICP)
    add_subdirectory(kiss-icp/cpp/kiss_icp)
    list(APPEND SRCS kiss_icp.cpp)
    list(APPEND LIBS kiss_icp_core)
endif()

# TODO: Figure out python bindings with these

# ------------------------- Put all wrappers into a target ------------------------- #
# find all source and header files

# Create Library
add_library(${LIB_NAME} "${SRCS}") 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_link_libraries(${LIB_NAME} PUBLIC ${LIBS})