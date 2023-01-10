cmake_minimum_required(VERSION 3.0)
PROJECT(gaussself)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${Eigen3_DIR})
INCLUDE_DIRECTORIES(${OpenCV_DIR})

add_executable(main gaussNewton.cc)

target_link_libraries(main ${OpenCV_LIBS})
