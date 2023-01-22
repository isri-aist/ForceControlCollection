add_library(ForceColl
  Contact.cpp
  )
target_link_libraries(ForceColl PUBLIC mc_rtc::mc_rtc_utils)
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(ForceColl PUBLIC ${catkin_LIBRARIES})
else()
  target_link_libraries(ForceColl PUBLIC qp_solver_collection::QpSolverCollection)
endif()
target_include_directories(ForceColl PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ForceColl PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(ForceColl PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS ForceColl
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ForceColl DESTINATION "${INCLUDE_INSTALL_DIR}")
