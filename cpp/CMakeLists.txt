# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(LIBS Eigen3::Eigen)
set(LIB_NAME evalio)
set(DEF "")
set(RESULTS "")

set(PIPELINES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/evalio/pipelines-src")

# ------------------------- Import all pipelines ------------------------- #
function(find_pipeline SRC_DIR TARGET_NAME DEF_NAME)
    if(EXISTS "${PIPELINES_SRC}/${SRC_DIR}/")
        message("----------------------- Adding ${SRC_DIR} -----------------------")
        add_subdirectory(${PIPELINES_SRC}/${SRC_DIR})
        set(LIBS ${LIBS} ${TARGET_NAME} PARENT_SCOPE)
        set(DEF ${DEF} ${DEF_NAME} PARENT_SCOPE)
        set(RESULTS ${RESULTS} "-- ✓ ${DEF_NAME}\n" PARENT_SCOPE)
        message("----------------------- Finished ${SRC_DIR} -----------------------")
    else()
        set(RESULTS ${RESULTS} "-- x ${DEF_NAME}\n" PARENT_SCOPE)
    endif()
endfunction()

message("################### Searching for pipelines ###################")
find_pipeline(kiss-icp/cpp/kiss_icp  kiss_icp_pipeline EVALIO_KISS_ICP)
find_pipeline(LIO-SAM                lio_sam           EVALIO_LIO_SAM)
message("###################    Pipeline Results     ###################")
if(NOT "${RESULTS}" STREQUAL "")
    message(" ")
    message(${RESULTS})
endif()
message("###############################################################")

# ------------------------- Put all wrappers into a target ------------------------- #
# Create Library
add_library(${LIB_NAME} INTERFACE) 
add_library(${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

# Set Library Properties
target_include_directories(${LIB_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${LIB_NAME} INTERFACE ${LIBS})
target_compile_definitions(${LIB_NAME} INTERFACE ${DEF})
set_target_properties(${LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)