cmake_minimum_required(VERSION 3.0)

project(cvTEST)


find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(1.interpolation  1.interpolation.cc)
add_executable(2.undistortImage  2.undistortImage.cc)


target_link_libraries(1.interpolation ${OpenCV_LIBS})
target_link_libraries(2.undistortImage ${OpenCV_LIBS})
