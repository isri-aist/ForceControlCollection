if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_ForceColl_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest ForceColl)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_ForceColl_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} ForceColl)
  endfunction()
endif()

set(ForceColl_gtest_list
  TestContact
  TestWrenchDistribution
  )

foreach(NAME IN LISTS ForceColl_gtest_list)
  add_ForceColl_test(${NAME})
endforeach()
