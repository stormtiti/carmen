cmake_minimum_required(VERSION 2.8.3)
project(libhaloc)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    tf
    cv_bridge
    image_geometry
    std_msgs
    cmake_modules)

# Dependencies - Eigen:
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include LIBRARIES haloc)

# Add the Image Hashing library
add_library(haloc
            src/lc.cpp
            src/image.cpp
            src/hash.cpp)
target_link_libraries(haloc ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}
opencv_nonfree)

# Include directories
include_directories(${catkin_INCLUDE_DIRS} include)