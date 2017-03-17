cmake_minimum_required(VERSION 2.8.3)
project(slam_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
        dynamic_reconfigure
        image_transport
        message_filters
        tf
        nav_msgs
        visualization_msgs
        gazebo_ros
        robot_state_publisher
        laser_geometry
        pointmatcher_ros
        #grid_map_ros
        #grid_map_cv
        #grid_map_msgs
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# add eigen 3
find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES slam_sim
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
        DEPENDS  roscpp geometry_msgs
)

SET(ROS_ROOT_PATH $ENV{ROS_ROOT})

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_CONFIG_DIR}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# add library: ROS diff drive plugin
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(robot_ros_utils src/gazebo_plugins/robot_ros_utils.cpp)
target_link_libraries(robot_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(robot_ros_diff_drive src/gazebo_plugins/robot_ros_diff_drive.cpp)
target_link_libraries(robot_ros_diff_drive robot_ros_utils ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# test apps
#add_executable(laser_scan_test src/laser_scan_test.cpp)
#target_link_libraries(laser_scan_test ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
#        "${CMAKE_THREAD_LIBS_INIT}" ${libpointmatcher_LIBRARIES}
#        ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(pcl_test src/test/pcl/pcl_test.cpp)
target_link_libraries(pcl_test ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
        ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

#add_executable(voxel_grid_filter src/test/pcl/voxel_grid_filter.cpp)
#target_link_libraries(voxel_grid_filter ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
#        ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})