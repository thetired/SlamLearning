cmake_minimum_required(VERSION 3.0)

project(orb)
set(CMAKE_CXX_FLAGS "-std=c++14 -mfma")
#add_definitions(-w)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIRS})

add_executable(orbCV orb_cv.cc)
add_executable(orbself orb_self.cc)


target_link_libraries(orbCV ${OpenCV_LIBS})
target_link_libraries(orbself ${OpenCV_LIBS})
