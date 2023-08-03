add_custom_target (tests)

set (test_name main)

include_directories(
  .
  ../lib/grid_tied_allocation/include
)

link_directories(
  ../lib/grid_tied_allocation/lib

)

link_libraries(
  grid_tied_allocation
)

add_executable (${test_name} ${test_name}.cc)

target_link_libraries(${test_name} grid_tied_allocation)


