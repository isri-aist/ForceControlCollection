if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_WrenchDistribution_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest WrenchDistribution)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_WrenchDistribution_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} WrenchDistribution)
  endfunction()
endif()

set(WrenchDistribution_gtest_list
  TestWrenchDistribution
  )

foreach(NAME IN LISTS WrenchDistribution_gtest_list)
  add_WrenchDistribution_test(${NAME})
endforeach()
