#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import time

class TurnLeft:
    def __init__(self):
        rospy.init_node('turn_left', anonymous=True)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # Hz
        self.turn()

    def turn(self):
        # Create a Twist message and set angular velocity
        turn_msg = Twist()
        turn_msg.angular.z = math.radians(45)  # Set rotation speed to 45 degrees per second

        # Calculate the time required to complete a 90 degree turn
        turn_time = math.radians(90) / turn_msg.angular.z

        # Start turning
        start_time = time.time()
        while time.time() - start_time < turn_time:
            self.vel_pub.publish(turn_msg)
            self.rate.sleep()

        # Stop the robot after turning
        turn_msg.angular.z = 0
        self.vel_pub.publish(turn_msg)
        rospy.signal_shutdown("Completed turn_left")

if __name__ == '__main__':
    try:
        TurnLeft()
    except rospy.ROSInterruptException:
        pass

