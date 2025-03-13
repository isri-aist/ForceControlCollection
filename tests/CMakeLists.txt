find_package(GTest REQUIRED)
include(GoogleTest)

set(ForceColl_gtest_list
  TestContact
  TestWrenchDistribution
)

# Prevents discovery failure before install
# No effect in CMake < 3.18
set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)

function(add_ForceColl_test NAME)
  add_executable(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC GTest::gtest ForceColl)
  gtest_discover_tests(${NAME})
endfunction()

foreach(NAME IN LISTS ForceColl_gtest_list)
  add_ForceColl_test(${NAME})
endforeach()
