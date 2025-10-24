# Turn off deprecated cmake warning for pipelines
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(PIPELINES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/pipelines-src")

# Force all targets to be compiled with -fPIC
# This is required for Python bindings to work properly
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
set(LIBS evalio Eigen3::Eigen) # targets to link to
set(DEF "") # definitions to pass to the python module
set(RESULTS "") # for printing results


# ------------------------- Make Python Bindings ------------------------- #
find_package(
    Python
    3.11
    REQUIRED
    COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule
)

FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.9.2
    FIND_PACKAGE_ARGS
        CONFIG # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(nanobind)

# Add the Python module
nanobind_add_module(_cpp STABLE_ABI NB_STATIC main.cpp)
target_link_libraries(_cpp PRIVATE ${LIBS})
target_compile_definitions(_cpp PRIVATE ${DEF})
install(TARGETS _cpp DESTINATION evalio)

# handle stubs
nanobind_add_stub(
    evalio_stubs
    INSTALL_TIME
    MODULE evalio._cpp
    RECURSIVE

    OUTPUT_PATH evalio
    OUTPUT
        evalio/_cpp/__init__.pyi
        evalio/_cpp/pipelines.pyi
        evalio/_cpp/types.pyi
        evalio/_cpp/helpers.pyi
)

# install licenses as well
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../licenses DESTINATION .)
