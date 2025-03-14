add_library(ForceColl
  Contact.cpp
  WrenchDistribution.cpp
)

target_include_directories(ForceColl PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ForceColl PUBLIC cxx_std_17)

if(USE_ROS2)
  target_link_libraries(ForceColl PUBLIC
    rclcpp::rclcpp
  )
endif()

target_link_libraries(ForceColl PUBLIC 
  mc_rtc::mc_rtc_utils
  mc_rtc::mc_rtc_gui
  qp_solver_collection::QpSolverCollection
)

if(BUILD_SHARED_LIBS)
  set_target_properties(ForceColl PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS ForceColl
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ForceColl DESTINATION include)
