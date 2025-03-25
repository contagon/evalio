# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(LIBS evalio Eigen3::Eigen)
set(DEF "")
set(RESULTS "")

set(PIPELINES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/pipelines-src")

# ------------------------- Helpers ------------------------- #
# Extract version of a project which was previously included with add_subdirectory().
# https://stackoverflow.com/a/49551280
function(subproject_version SRC_DIR VERSION_VAR)
    # Read CMakeLists.txt for subproject and extract project() call(s) from it.
    file(
        STRINGS
        "${PIPELINES_SRC}/${SRC_DIR}/CMakeLists.txt"
        project_calls
        REGEX "[ \t]*project\\("
    )
    # For every project() call try to extract its VERSION option
    foreach(project_call ${project_calls})
        string(
            REGEX MATCH
            "VERSION[ ]+([^ )]+)"
            version_param
            "${project_call}"
        )
        if(version_param)
            set(version_value "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    if(version_value)
        set(${VERSION_VAR} "${version_value}" PARENT_SCOPE)
    else()
        message("WARNING: Cannot extract version for '${SRC_DIR}'")
    endif()
endfunction(subproject_version)

# iterates over pipeline folders to find them
function(find_pipeline SRC_DIR TARGET_NAME DEF_NAME)
    if(EXISTS "${PIPELINES_SRC}/${SRC_DIR}/")
        message(
            "----------------------- Adding ${SRC_DIR} -----------------------"
        )
        add_subdirectory(${PIPELINES_SRC}/${SRC_DIR})
        set(LIBS ${LIBS} ${TARGET_NAME} PARENT_SCOPE)
        subproject_version(${SRC_DIR} ${DEF_NAME}_version)
        set(DEF ${DEF} ${DEF_NAME}=${${DEF_NAME}_version} PARENT_SCOPE)
        set(RESULTS
            ${RESULTS}
            "-- ✓ ${DEF_NAME} ${${DEF_NAME}_version}\n"
            PARENT_SCOPE
        )
        message(
            "----------------------- Finished ${SRC_DIR} -----------------------"
        )
    else()
        set(RESULTS ${RESULTS} "-- x ${DEF_NAME}\n" PARENT_SCOPE)
    endif()
endfunction()

# ------------------------- Import all pipelines ------------------------- #
message("################### Searching for pipelines ###################")
find_pipeline(kiss-icp/cpp/kiss_icp  kiss_icp_pipeline EVALIO_KISS_ICP)
find_pipeline(LIO-SAM                lio_sam           EVALIO_LIO_SAM)
message("###################    Pipeline Results     ###################")
if(NOT "${RESULTS}" STREQUAL "")
    string(STRIP "${RESULTS}" RESULTS)
    message(${RESULTS})
endif()
message("###############################################################")

# ------------------------- Make Python Bindings ------------------------- #
set(PYBIND11_FINDPYTHON ON)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    # FIND_PACKAGE_ARGS # uses find_package first, git if it fails
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
