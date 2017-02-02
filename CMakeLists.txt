cmake_minimum_required(VERSION 2.8.3)
project(slam_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES slam_sim
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
        DEPENDS  roscpp geometry_msgs
)

SET(ROS_ROOT_PATH $ENV{ROS_ROOT})

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_CONFIG_DIR}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
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
