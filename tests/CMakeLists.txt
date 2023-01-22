if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_ForceControlCollection_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest ForceControlCollection)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_ForceControlCollection_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} ForceControlCollection)
  endfunction()
endif()

set(ForceControlCollection_gtest_list
  TestWrenchDistribution
  )

foreach(NAME IN LISTS ForceControlCollection_gtest_list)
  add_ForceControlCollection_test(${NAME})
endforeach()
