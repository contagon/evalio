# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(LIBS evalio Eigen3::Eigen)
set(DEF "")
set(RESULTS "")

set(PIPELINES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/pipelines-src")

# ------------------------- Import all pipelines ------------------------- #
function(find_pipeline SRC_DIR TARGET_NAME DEF_NAME)
    if(EXISTS "${PIPELINES_SRC}/${SRC_DIR}/")
        message(
            "----------------------- Adding ${SRC_DIR} -----------------------"
        )
        add_subdirectory(${PIPELINES_SRC}/${SRC_DIR})
        set(LIBS ${LIBS} ${TARGET_NAME} PARENT_SCOPE)
        set(DEF ${DEF} ${DEF_NAME} PARENT_SCOPE)
        set(RESULTS ${RESULTS} "-- ✓ ${DEF_NAME}\n" PARENT_SCOPE)
        message(
            "----------------------- Finished ${SRC_DIR} -----------------------"
        )
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

# ------------------------- Make Python Bindings ------------------------- #
set(PYBIND11_FINDPYTHON ON)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    FIND_PACKAGE_ARGS # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(_cpp main.cpp)
target_link_libraries(_cpp PRIVATE ${LIBS})
target_compile_definitions(_cpp PRIVATE ${DEF})
install(TARGETS _cpp DESTINATION evalio)

# install licenses as well
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../licenses DESTINATION .)

# Build stubs
# TODO: Something about this breaks python 3.12 and 3.13 builds
# add_custom_command(
#     TARGET _cpp
#     POST_BUILD
#     COMMAND
#         "${Python_EXECUTABLE}" -m pybind11_stubgen _cpp -o
#         "${CMAKE_CURRENT_BINARY_DIR}" --numpy-array-wrap-with-annotated
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#     COMMENT "Generating pybind11 stubs"
#     VERBATIM
# )
# install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/_cpp DESTINATION evalio)
