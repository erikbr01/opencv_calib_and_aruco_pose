
add_executable(opencv_calib_and_aruco_pose 
src/main.cpp
src/aruco_functions.cpp
src/calibrator.cpp
)

target_include_directories(opencv_calib_and_aruco_pose PUBLIC include)

target_link_libraries(opencv_calib_and_aruco_pose PRIVATE ${OpenCV_LIBS})


