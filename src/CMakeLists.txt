cmake_minimum_required(VERSION 3.0.0)
project(multi_lidar_calibrator VERSION 0.1.0)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    pcl_conversions
    )

catkin_package(CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

SET(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_STANDARD 11)

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar Calibrator
add_library(multi_lidar_calibrator_lib SHARED
    src/CalibrationParam.cpp
    include/CalibrationParam.h
    src/multi_lidar_calibrator.cpp
    include/multi_lidar_calibrator.h
    src/multi_lidar_calibrator_asynchronous.cpp
    include/multi_lidar_calibrator_asynchronous.h)

target_include_directories(multi_lidar_calibrator_lib PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(multi_lidar_calibrator_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_executable(multi_lidar_calibrator
    src/multi_lidar_calibrator_node.cpp
    )

target_include_directories(multi_lidar_calibrator PRIVATE
    include)

target_link_libraries(multi_lidar_calibrator
    multi_lidar_calibrator_lib
    libjsoncpp.a)

add_executable(multi_lidar_calibrator_asynchronous
    src/multi_lidar_calibrator_asynchronous_node.cpp
    )

target_include_directories(multi_lidar_calibrator_asynchronous PRIVATE
    include)

target_link_libraries(multi_lidar_calibrator_asynchronous
    multi_lidar_calibrator_lib
    libjsoncpp.a)