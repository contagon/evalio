# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(SRCS "")
set(LIBS Eigen3::Eigen)
set(LIB_NAME evalio_cpp)

# ------------------------- Import all pipelines ------------------------- #
# options
option(EVALIO_BUILD_ALL "Build all included pipelines" ON)
option(EVALIO_KISS_ICP "Build KISS-ICP" OFF)
option(EVALIO_LIO_SAM "Build Lio-Sam" OFF)

# KISS-ICP
if(EVALIO_BUILD_ALL OR EVALIO_KISS_ICP)
    message(STATUS "Building KISS-ICP")
    add_subdirectory(evalio/pipelines/kiss-icp/cpp/kiss_icp)
    list(APPEND LIBS kiss_icp_pipeline)
endif()

# LIO-SAM
if(EVALIO_BUILD_ALL OR EVALIO_LIO_SAM)
    message(STATUS "Building LIO-SAM")
    add_subdirectory(evalio/pipelines/lio-sam/)
    list(APPEND LIBS lio_sam)
endif()

# ------------------------- Put all wrappers into a target ------------------------- #
# make position_indepent code for python wrapper
foreach(L IN LISTS LIBS)
    set_target_properties(${L} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
endforeach()


# Create Library
add_library(${LIB_NAME} "${SRCS}") 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${LIB_NAME} PUBLIC ${LIBS})
target_compile_definitions(${LIB_NAME} PUBLIC cxx_standard_20)
set_target_properties(${LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)