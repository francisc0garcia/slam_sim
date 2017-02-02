#!/usr/bin/env python

import sys
import time
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist


class robot_move_test:

    def __init__(self):
        # Init ros node
        rospy.init_node('robot_cmd_vel')

        self.enable_controller = False

        self.speed_linear = 0.5
        self.speed_angular = 0.6

        # create a publisher for command velocity
        self.cmd_vel_pub_robot_1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_pub_robot_2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_pub_robot_3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_pub_robot_4 = rospy.Publisher('/robot_4/cmd_vel', Twist, queue_size=1)

        self.sub_twist = rospy.Subscriber('/cmd_vel', Twist, self.process_twist_message, queue_size=1)

        # define rate of transmission: 10 hz
        self.rate = rospy.Rate(10.0)

        # send commands at 10 hz
        while not rospy.is_shutdown():
            # --------------------------------------------
            # move robot forward, rotate, stop and return.
            # --------------------------------------------

            if self.enable_controller:
                self.move_forward()
                time.sleep(5)

                self.stop_robot()
                time.sleep(1)

                self.move_backward()
                time.sleep(5)

            self.rate.sleep()

    # move forward robot
    def move_forward(self):
        # create Twist message
        cmd = Twist()
        cmd.linear.x = self.speed_linear
        cmd.angular.z = 0.0

        # Send command
        self.send_command(cmd)

    # move backward robot
    def move_backward(self):
        # create Twist message
        cmd = Twist()
        cmd.linear.x = -self.speed_linear
        cmd.angular.z = 0.0

        # Send command
        self.send_command(cmd)

    # rotate to left
    def rotate_left(self):
        # create Twist message
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.speed_angular

        # Send command
        self.send_command(cmd)

    # rotate to right
    def rotate_right(self):
        # create Twist message
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -self.speed_angular

        # Send command
        self.send_command(cmd)

    # stop robot: set linear and angular velocity to zero
    def stop_robot(self):
        # create Twist message
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        # Send command
        self.send_command(cmd)

    def send_command(self, cmd):
        # Send command
        self.cmd_vel_pub_robot_1.publish(cmd)
        self.cmd_vel_pub_robot_2.publish(cmd)
        self.cmd_vel_pub_robot_3.publish(cmd)
        self.cmd_vel_pub_robot_4.publish(cmd)

    def process_twist_message(self, twist_msg):
        if twist_msg.linear.x > 0:
            self.enable_controller = True

def main(args):

    ic = robot_move_test()
    rospy.init_node('move_robot_test', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)