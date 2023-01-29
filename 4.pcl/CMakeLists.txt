cmake_minimum_required(VERSION 3.0)

project(pclFunction)

find_package(PCL REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

message(WARNING  ${PCL_INCLUDE_DIRS} )
add_executable(passThrough filter.cc)

target_link_libraries(passThrough ${PCL_LIBRARIES})