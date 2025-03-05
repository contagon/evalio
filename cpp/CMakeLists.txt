# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(LIBS Eigen3::Eigen)
set(LIB_NAME evalio_cpp)
set(DEF "")

# ------------------------- Import all pipelines ------------------------- #
function(find_pipeline PIPELINE_NAME TARGET_NAME DEF_NAME)
    find_package(${PIPELINE_NAME} QUIET)
    if(TARGET ${TARGET_NAME})
        message(STATUS "✓ ${PIPELINE_NAME}")
        list(APPEND LIBS ${TARGET_NAME})
        list(APPEND DEF ${DEF_NAME})
    else()
        message(STATUS "x ${PIPELINE_NAME}")
    endif()
endfunction()

message("################### Searching for pipelines ###################")
find_pipeline(KissICP  kiss_icp_pipeline EVALIO_KISS_ICP)
find_pipeline(LioSam   lio_sam           EVALIO_LIO_SAM)
message("###############################################################")

# ------------------------- Put all wrappers into a target ------------------------- #
# make position_independent code for python wrapper
foreach(L IN LISTS LIBS)
    set_target_properties(${L} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
endforeach()

# Create Library
add_library(${LIB_NAME} INTERFACE) 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_include_directories(${LIB_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${LIB_NAME} INTERFACE ${LIBS})
target_compile_definitions(${LIB_NAME} INTERFACE ${DEF})
set_target_properties(${LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)