project(imutest)
cmake_minimum_required(VERSION 2.8)

# Look for PCL
find_package(PCL 1.6 REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++11")

# Look for OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(imutest imutest.cpp)
target_link_libraries(imutest ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS})


