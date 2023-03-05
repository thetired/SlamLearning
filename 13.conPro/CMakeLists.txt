cmake_minimum_required(VERSION 3.0)
project(conPro)

SET(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(main main.cc)

target_link_libraries(main   -lpthread)