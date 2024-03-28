#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose():
    rospy.init_node('set_initial_pose')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Wait for publisher to initialize

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = 0.0000  # Set initial x position
    initial_pose.pose.pose.position.y = 0.0000  # Set initial y position
    initial_pose.pose.pose.orientation.w = 1.0  # Set initial orientation (quaternion)

    pub.publish(initial_pose)
    rospy.loginfo("Initial pose set to x: {}, y: {}".format(initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y))
    rospy.sleep(1) 

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
