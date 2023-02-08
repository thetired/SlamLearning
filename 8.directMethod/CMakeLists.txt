cmake_minimum_required(VERSION 3.0)

project(directmethod)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

message(STATUS   ${EIGEN3_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS}
                                            ${EIGEN3_INCLUDE_DIR}
                                            ${SOPHUS_INCLUDE_DIRECTORIES}
                                            ${CERES_INCLUDE_DIRECTORIES})
                                            
add_executable( main  main.cc)
add_executable(ceresDirect ceres.cc)
add_executable(ceresTest ceres_test.cc)


target_link_libraries(main ${OpenCV_LIBS} ${SOPHUS_LIBRARIES})

target_link_libraries(ceresDirect ${OpenCV_LIBS} ${SOPHUS_LIBRARIES}   ${CERES_LIBRARIES})  
target_link_libraries(ceresTest ${OpenCV_LIBS} ${SOPHUS_LIBRARIES}   ${CERES_LIBRARIES})  


