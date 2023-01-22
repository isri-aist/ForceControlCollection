add_library(ForceControlCollection
  Contact.cpp
  )
target_link_libraries(ForceControlCollection PUBLIC mc_rtc::mc_rtc_utils)
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(ForceControlCollection PUBLIC ${catkin_LIBRARIES})
else()
  target_link_libraries(ForceControlCollection PUBLIC qp_solver_collection::QpSolverCollection)
endif()
target_include_directories(ForceControlCollection PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ForceControlCollection PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(ForceControlCollection PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS ForceControlCollection
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ForceControlCollection DESTINATION "${INCLUDE_INSTALL_DIR}")
