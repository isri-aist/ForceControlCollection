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