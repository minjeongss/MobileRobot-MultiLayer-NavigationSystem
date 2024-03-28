#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
import math
from apriltag_ros.msg import AprilTagDetectionArray

class cmd_pose:

    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(5)  # Set loop frequency to 5 Hz

    def callback(self, tag):
        if tag.detections:
            try:
                self.listener.waitForTransform('/base_link', '/num1', rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.listener.lookupTransform('/base_link', '/num1', rospy.Time(0))
                x = trans[0]
                y = trans[1]
                z = trans[2]

                distance = math.sqrt(x ** 2 + y ** 2)
                target_distance = 1.6# Target distance in meters
                angular = math.atan2(y, x)

                linear = 0.35 * (distance - target_distance)
                
                if distance >1.7:
                    linear = 0.35
                
                if abs(distance - target_distance) < 0.01:  # If the robot is close to the target distance
                    linear = 0

                if abs(angular) < 0.003:  # If the robot is facing the target direction
                    angular = 0

                # Velocity command
                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                
                self.vel_pub.publish(cmd)

                # Check if the robot has reached the target position and orientation
                if linear == 0 and angular == 0:
                    rospy.sleep(1)
                    rospy.signal_shutdown("Target reached")
                    print("Target reached, shutting down node")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform")
        else:
            print("NO DATA DETECTIONS!!")

if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")