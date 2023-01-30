cmake_minimum_required(VERSION 3.0)

project(pclFunction)

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

include_directories( ${EIGEN_INCLUDE_DIRS})

#message(WARNING  ${PCL_INCLUDE_DIRS} )
add_executable(passThrough filter.cc)

add_executable(massCenter massCenter.cc)

add_executable(boundary  boundary.cc)



target_link_libraries(passThrough ${PCL_LIBRARIES})
target_link_libraries(massCenter ${PCL_LIBRARIES})
target_link_libraries(boundary ${PCL_LIBRARIES})

