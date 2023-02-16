cmake_minimum_required(VERSION 3.0)
project(dense_monulaer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAEK_CXX_FLAGS "-std=c++11 -march=native -O3")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

set(INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR} ${Sophus_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${INCLUDE_DIRS})


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES})

add_executable(dense_mapping main.cc)

target_link_libraries(dense_mapping ${THIRD_PARTY_LIBS})