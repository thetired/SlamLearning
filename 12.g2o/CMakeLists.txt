cmake_minimum_required(VERSION 3.0)
project(g2o)

set(CMAKE_BUILD_TYPE  Release)
set(CMAKE_CXX_FLAGS  "-std=c++14 -O3"   )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

message(STATUS ${G2O_INCLUDE_DIRS} )

include_directories("/usr/include/eigen3")

add_executable(curveFitting 1.curveFitting.cc)
add_executable(pose_estimation 2.pnp.cc)
add_executable(pose_estimation_3d_3d  3.p3p.cc)




target_link_libraries(curveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
target_link_libraries(pose_estimation ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
target_link_libraries(pose_estimation_3d_3d ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})


