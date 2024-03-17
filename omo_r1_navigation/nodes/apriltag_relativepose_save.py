#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import json

class AprilTagListener:
    def __init__(self):
        rospy.init_node('apriltag_listener', anonymous=True)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.tags_data = {}

    def tag_callback(self, data):
        if data.detections:
            tag=data.detections[0]
            tag_id=tag.id[0]
            tag_pose = tag.pose.pose.pose.position
            self.tags_data[tag_id] = {
                'x': tag_pose.x,
                'y': tag_pose.y,
                'z': tag_pose.z
            }
            print("---------tag id: ",tag_id,"---------")
            self.save_to_json("poss.json")

    def save_to_json(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.tags_data, f, indent=4)
        rospy.loginfo("AprilTag positions saved to {}".format(filename))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = AprilTagListener()
        listener.run()
    except rospy.ROSInterruptException:
        pass

