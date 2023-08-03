cmake_minimum_required(VERSION 3.5)

project(grid_tied_allocation VERSION 1.0)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

if (NOT DEFINED CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif ()

# set build attributions
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-w)
    add_compile_options(-std=c++11)
    add_compile_options(-Wall -O2 -c -fpermissive -g) #g
    add_compile_options(-D _GLIBCXX_USE_NANOSLEEP)
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)


include_directories(
  include
  ../lib/nlopt/include
  ../lib/osqp/include/osqp
)


link_directories(
	../lib/nlopt/lib
  ../lib/osqp/lib
)


link_libraries(
	rt
	pthread
	nlopt
  osqpstatic
)

set(library_name grid_tied_allocation)

add_library(${library_name}
  src/common.cc
  src/dp_cost.cc
  src/graph_point.cc
  src/grid_tied_allocation.cc
  src/heuristic_optimizer.cc
  src/nlopt_optimizer.cc
  src/optimizer_config.cc
  src/osqp_optimizer.cc
  src/small_load_assignment.cc
)

target_link_libraries(${library_name} rt pthread)

install ( TARGETS ${PROJECT_NAME}
          RUNTIME DESTINATION ${RELATIVE_INSTALL_BIN_DIR}
          LIBRARY DESTINATION ${RELATIVE_INSTALL_LIB_DIR}
          ARCHIVE DESTINATION ${RELATIVE_INSTALL_LIB_DIR}
          INCLUDES DESTINATION ${RELATIVE_INSTALL_INCLUDE_DIR})


install(DIRECTORY include/
        DESTINATION include/)


if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_custom_target(upload
    COMMAND rsync -arh --progress
          ${CMAKE_CURRENT_SOURCE_DIR}/bin/* # source
          root@192.178.111.46:/mnt/mmc/algorithms/ # destination
    COMMENT "uploading algorithm"
    USES_TERMINAL
)
