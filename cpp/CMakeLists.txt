# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(LIBS Eigen3::Eigen)
set(LIB_NAME evalio_cpp)
set(DEF "")

# ------------------------- Import all pipelines ------------------------- #
# KISS-ICP
find_package(KissICP QUIET)
if(TARGET kiss_icp_pipeline)
    message(STATUS "Found KISS-ICP")
    list(APPEND LIBS kiss_icp_pipeline)
    list(APPEND DEF EVALIO_KISS_ICP)
else()
    message(STATUS "KISS-ICP not found")
endif()

# LIO-SAM
find_package(LioSam QUIET)
if(TARGET lio_sam)
    message(STATUS "Found LIO-SAM")
    list(APPEND LIBS lio_sam)
    list(APPEND DEF EVALIO_LIO_SAM)
else()
    message(STATUS "LIO-SAM not found")
endif()

# ------------------------- Put all wrappers into a target ------------------------- #
# make position_independent code for python wrapper
# foreach(L IN LISTS LIBS)
#     set_target_properties(${L} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
# endforeach()


# Create Library
add_library(${LIB_NAME} INTERFACE) 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_include_directories(${LIB_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${LIB_NAME} INTERFACE ${LIBS})
target_compile_definitions(${LIB_NAME} INTERFACE ${DEF})
set_target_properties(${LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)