# ------------------------- Library ------------------------- #
add_library(evalio INTERFACE)
target_include_directories(
    evalio
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

# ------------------------- Install ------------------------- #
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# config file
install(TARGETS evalio EXPORT EvalioConfig)
install(EXPORT EvalioConfig DESTINATION "${CMAKE_INSTALL_DATADIR}/evalio/cmake")

# headers
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
    PATTERN "*.h"
)

# version file
write_basic_package_version_file(
    EvalioConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/EvalioConfigVersion.cmake
    DESTINATION "${CMAKE_INSTALL_DATADIR}/evalio/cmake"
)
