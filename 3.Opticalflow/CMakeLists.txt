cmake_minimum_required(VERSION 3.0)

project(OpticalFlow)


find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

INCLUDE_DIRECTORIES("/usr/include/eigen3/")





add_executable(main  main.cc  OpticalFlowTracker.cc )
target_link_libraries(main ${OpenCV_LIBS})