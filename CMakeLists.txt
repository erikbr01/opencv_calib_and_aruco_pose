cmake_minimum_required(VERSION 3.16)
project(cv_aruco_demo VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)

# Find requirements
find_package(OpenCV REQUIRED)


add_subdirectory(lib)
add_subdirectory(app)

