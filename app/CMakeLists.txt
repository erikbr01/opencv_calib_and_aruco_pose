
add_executable(cv_aruco_demo 
src/main.cpp
src/aruco_functions.cpp
src/calibrator.cpp
)

target_include_directories(cv_aruco_demo PUBLIC include)

target_link_libraries(cv_aruco_demo PRIVATE ${OpenCV_LIBS})


