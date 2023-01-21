add_library(WrenchDistribution
  Contact.cpp
  )
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_include_directories(WrenchDistribution PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(WrenchDistribution PUBLIC
    ${catkin_LIBRARIES}
  )
else()
  target_link_libraries(WrenchDistribution PUBLIC qp_solver_collection::QpSolverCollection)
endif()
target_include_directories(WrenchDistribution PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(WrenchDistribution PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(WrenchDistribution PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS WrenchDistribution
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/WrenchDistribution DESTINATION "${INCLUDE_INSTALL_DIR}")
