#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class ApriltagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector')
        self.tag_sub=rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_callback)
        tag=[]

    def tag_callback(self,data):
        if not data.detections:
            rospy.logwarn("[ERROR] No AprilTag detect!")
        else:
            # AprilTag 정보 받아오는 부분
            tag=data.detections[0]
            tag_id=tag.id[0]
            tag_x=tag.pose.pose.pose.position.x
            tag_y=tag.pose.pose.pose.position.y
            print("ID: ",tag_id)
            print("x: ",tag_x,"y: ",tag_y)

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    detector=ApriltagDetector()
    detector.run()
