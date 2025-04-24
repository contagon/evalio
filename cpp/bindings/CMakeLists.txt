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
    GIT_TAG v2.7.0
    # FIND_PACKAGE_ARGS # uses find_package first, git if it fails
)
FetchContent_MakeAvailable(nanobind)

# Add the Python module
nanobind_add_module(_cpp STABLE_ABI NB_STATIC main.cpp)
target_link_libraries(_cpp PRIVATE ${LIBS})
target_compile_definitions(_cpp PRIVATE ${DEF})
install(TARGETS _cpp DESTINATION evalio)

# handle stubs
function(module_subs MOD FILE)
    message(STATUS "Adding ${MOD} stubs")
    nanobind_add_stub(
        ${MOD}_stubs
        INSTALL_TIME
        MODULE evalio.${MOD}
        OUTPUT evalio/_cpp/${FILE} # install directly to final location
        PYTHON_PATH "."
    )
endfunction()

# option for github actions
# see ci.yml
option(EVALIO_PYTHON_STUBS "Build Python bindings with stubs" ON)
if(EVALIO_PYTHON_STUBS)
    module_subs(_cpp           __init__.pyi)
    module_subs(_cpp.pipelines pipelines.pyi)
    module_subs(_cpp.types     types.pyi)
endif()

# install licenses as well
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../licenses DESTINATION .)
