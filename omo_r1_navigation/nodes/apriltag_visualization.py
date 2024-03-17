#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publish_markers():
    rospy.init_node('marker_publisher_node', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # Adjust the publish rate as needed

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        # Create a marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = 0
        marker.pose.position.x = 0.2522
        marker.pose.position.y = -0.1519
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Add the marker to the array
        marker_array.markers.append(marker)

        # Publish the MarkerArray
        marker_pub.publish(marker_array)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass

