add_executable(helloCeres 1HelloCeres.cc)

add_executable(numerical 2NumericalD.cc)


find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

target_link_libraries(helloCeres ${CERES_LIBRARIES} glog)
target_link_libraries(numerical ${CERES_LIBRARIES} glog)

