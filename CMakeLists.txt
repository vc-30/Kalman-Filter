project(kalman_filter)
cmake_minimum_required(VERSION 3.5)


set(CMAKE_CXX_FLAGS "-std=c++11")
#For kf.hpp
find_package(Eigen3 REQUIRED)

#For matplotlibcpp
list(APPEND PYTHON_LIBRARIES "/usr/lib/python2.7/")
list(APPEND PYTHON_INCLUDE_DIRS "/usr/include/python2.7")

find_package(PythonLibs REQUIRED)


include_directories(
    include
    ${PYTHON_INCLUDE_DIRS}
    matplotlib-cpp
)

add_executable(
    kf_example
    src/main.cpp
)

target_link_libraries(
    kf_example
    ${PYTHON_LIBRARIES}
)


