#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf
import json

class AprilTagListener:
    def __init__(self):
        rospy.init_node('apriltag_listener', anonymous=True)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.tags_data = {}
        self.listener = tf.TransformListener()

    def tag_callback(self, data):
        if data.detections:
            tag=data.detections[0]
            tag_id=tag.id[0]
            tag_pose=tag.pose.pose.pose.position
            try:
                if(tag_id==1):
                    (trans, rot) = self.listener.lookupTransform('/map', '/elvfront', rospy.Time(0))
                elif(tag_id==3):
                    (trans, rot) = self.listener.lookupTransform('/map', '/floor3', rospy.Time(0))
                elif(tag_id==4):
                    (trans, rot) = self.listener.lookupTransform('/map', '/number4', rospy.Time(0))
                elif (tag_id==5):
                    (trans, rot) = self.listener.lookupTransform('/map', '/doorfront', rospy.Time(0))

                self.tags_data[tag_id] = {
                    'x': trans[0],
                    'y': trans[1],
                    'z': trans[2],
                }
                self.save_to_json("src/apriltag_pose.json")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform")
        else:
            print("NO DATA DETECTIONS!!")
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
