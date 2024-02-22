#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

class AprilTagMapper:
    def __init__(self):
        rospy.init_node('april_tag_mapper')

        self.bridge = CvBridge()
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.map_pub = rospy.Publisher('/april_tag_map', Image, queue_size=10)
        self.tf_listener = tf.TransformListener()

    def tag_callback(self, data):
        if not data.detections:
            rospy.logwarn("No AprilTag detected.")
            return
            
        # Assume only one tag is detected for simplicity
        tag = data.detections[0]
        tag_id = tag.id[0]

        # Get the 3D pose of the tag in the camera frame
        tag_pose_camera = PoseStamped()
        tag_pose_camera.header = tag.pose.header
        tag_pose_camera.pose = tag.pose.pose.pose
        print(tag_pose_camera)
	
        # Transform the pose to the map frame
        try:
            tag_pose_map = self.tf_listener.transformPose('map', tag_pose_camera)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to transform AprilTag pose to map frame.")
            return

        # Update the 2D map with the tag position
        map_image = self.update_map(tag_pose_map)

        # Publish the map with markers
        self.map_pub.publish(self.bridge.cv2_to_imgmsg(map_image, encoding="bgr8"))

    def update_map(self, tag_pose_map):
        # Load your 2D map image
        map_image = cv2.imread("/home/minjeong/catkin_ws/src/omo_r1_navigation/maps/map.yaml")

        # Convert the tag pose to pixel coordinates on the map
        tag_pixel_x = int(tag_pose_map.pose.position.x * map_resolution + map_origin_x)
        tag_pixel_y = int(tag_pose_map.pose.position.y * map_resolution + map_origin_y)

        # Draw a marker on the map at the tag position
        cv2.circle(map_image, (tag_pixel_x, tag_pixel_y), 5, (0, 255, 0), -1)
	
	#save map
	self.save_map(map_image)
	
        return map_image
	
    def save_map(self,map_image):
    	    cv2.imwrite("/home/minjeong/catkin_ws/src/omo_r1_navigation/maps/map_with_markers.pmg", map_image)
    	    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mapper = AprilTagMapper()
    mapper.run()

