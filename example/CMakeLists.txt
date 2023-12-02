cmake_minimum_required( VERSION 2.8 )
project( trajectoryError )

set(CMAKE_CXX_FLAGS "-std=c++14")
include_directories( "/usr/include/eigen3" )
find_package( Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )
find_package(fmt REQUIRED)
add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries( trajectoryError ${Pangolin_LIBRARIES}  fmt::fmt)