cmake_minimum_required(VERSION 3.0)

project(directmethod)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

message(STATUS   ${EIGEN3_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS}
                                            ${EIGEN3_INCLUDE_DIR}
                                            ${SOPHUS_INCLUDE_DIRECTORIES})
                                            
add_executable( main  main.cc)

target_link_libraries(main ${OpenCV_LIBS} ${SOPHUS_LIBRARIES})
