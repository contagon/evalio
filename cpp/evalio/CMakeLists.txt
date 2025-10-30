# ------------------------- Library ------------------------- #
add_library(evalio INTERFACE)
target_link_libraries(evalio INTERFACE Eigen3::Eigen)
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
install(TARGETS evalio EXPORT evalioConfig)
install(EXPORT evalioConfig DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/evalio")

# headers
# TODO: Verify this is handling nested conversion files correctly
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
    PATTERN "*.h"
)

# version file
write_basic_package_version_file(
    evalioConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/evalioConfigVersion.cmake
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/evalio"
)
