cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME force_control_collection)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/ForceControlCollection)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
option(USE_ROS2 "Use ROS2" OFF)

project(force_control_collection LANGUAGES CXX)

# Eigen
find_package(Eigen3 REQUIRED)

# mc_rtc
find_package(mc_rtc REQUIRED)

if(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(qp_solver_collection REQUIRED)
else()
  option(BUILD_TESTING "Build test" ON)
  find_package(qp_solver_collection REQUIRED)
endif()

add_subdirectory(src)

if(NOT USE_ROS2)
  install(EXPORT ${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    )

  # Write initial config cmake file
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in"
"
\@PACKAGE_INIT\@

include(\"\${CMAKE_CURRENT_LIST_DIR}/\@PROJECT_NAME\@Targets.cmake\")
include(\"\${CMAKE_CURRENT_LIST_DIR}/qp_solver_options.cmake\")

set(\@PROJECT_NAME\@_LIBRARIES \@PROJECT_NAME\@::ForceColl)
")

  include(CMakePackageConfigHelpers)
  # generate the config file that is includes the exports
  configure_package_config_file(${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    )

  # generate the version file for the config file
  # Extract version numbers from package.xml
  file(READ package.xml PACKAGE_XML)
  string(REGEX MATCH "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>" DIRTY_VERSION_STRING ${PACKAGE_XML})
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\1" FCC_MAJOR_VERSION "${DIRTY_VERSION_STRING}")
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\2" FCC_MINOR_VERSION "${DIRTY_VERSION_STRING}")
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\3" FCC_PATCH_VERSION "${DIRTY_VERSION_STRING}")
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${FCC_MAJOR_VERSION}.${FCC_MINOR_VERSION}.${FCC_PATCH_VERSION}"
    COMPATIBILITY AnyNewerVersion
    )

  # install config files
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    "${QP_SOLVER_OPTIONS_CMAKE_FILE}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(USE_ROS2)
  ament_export_include_directories(include)
  ament_export_libraries(ForceColl)
  ament_export_dependencies(
    rclcpp
    qp_solver_collection
  )
  ament_export_targets(ForceColl)
  ament_package()
endif() 