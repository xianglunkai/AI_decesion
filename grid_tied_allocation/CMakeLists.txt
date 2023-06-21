cmake_minimum_required(VERSION 3.5)

# set external environment variables path
# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/toolchain.cmake)

project(grid_tied_allocation VERSION 1.0)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# set build attributions
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-w)
    add_compile_options(-std=c++11)
    add_compile_options(-Wall -O2 -c) #g
    add_compile_options(-D _GLIBCXX_USE_NANOSLEEP)
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

link_libraries(
  rt
  pthread
)

include_directories(
  .
)

aux_source_directory(${CMAKE_SOURCE_DIR} SOURCE)

add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(${PROJECT_NAME}
  rt
  pthread
)


