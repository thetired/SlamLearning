cmake_minimum_required(VERSION 2.8)
project(vo1)

find_package(OpenCV 3  REQUIRED)

find_package(Sophus REQUIRED)

find_package(Ceres REQUIRED)



include_directories(    ${OpenCV_INCLUDE_DIRS}   "/usr/include/eigen3"  ${Sophus_INCLUDE_DIRS}   ${CERES_INCLUDE_DIRS})
add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})


add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d
        ${OpenCV_LIBS})

add_executable(pose_ceres  pose_estimation_ceres.cc)
target_link_libraries(pose_ceres
        ${OpenCV_LIBS}    ${CERES_LIBRARIES} )   

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES})