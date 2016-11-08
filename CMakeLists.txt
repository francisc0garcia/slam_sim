cmake_minimum_required(VERSION 2.8.3)
project(slam_sim)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        gazebo_ros
        robot_state_publisher
        )

include_directories(include)

include_directories(
        ${catkin_INCLUDE_DIRS}
)