cmake_minimum_required(VERSION 3.0)
project(carmen_dloop)

find_package(catkin REQUIRED)
# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=gnu++11")

SET(SRC_FILE
	${PROJECT_SOURCE_DIR}/src/DBoW2
	${PROJECT_SOURCE_DIR}/src/DUtils
	${PROJECT_SOURCE_DIR}/src/DVision
	${PROJECT_SOURCE_DIR}/src/DLoopDetector
	${PROJECT_SOURCE_DIR}/src/DUtilsCV)


SET(LIB_CPP_LIST ${})

FOREACH(dir ${SRC_FILE})
        aux_source_directory(${dir} CPPS)

        if(dir STREQUAL "DLoopDetector")
                file(GLOB CPPS RELATIVE "${dir}" "*.h")
        endif()

        list(APPEND LIB_CPP_LIST ${LIB_CPP_LIST} ${CPPS})
        #message(STATUS "${dir} contains ${CPPS}")
ENDFOREACH(dir ${SRC_FILE})



include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS OpenCV
)
include_directories(include)

add_library(carmen_dloop SHARED ${LIB_CPP_LIST})
target_link_libraries(carmen_dloop ${OpenCV_LIBS})






