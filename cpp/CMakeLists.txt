cmake_minimum_required(VERSION 2.8.3)
project(imageLidarAlign)

set(CMAKE_CXX_FLAGS "-std=c++14")

set(CMAKE_BUILD_TYPE debug)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_executable(imageLidarAlign src/imageLidarAlign.cpp) 
target_link_libraries(imageLidarAlign ${PCL_LIBRARIES} ${OpenCV_LIBS})
