#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
import math
from apriltag_ros.msg import AprilTagDetectionArray

from rospy import Rate

class cmd_pose:

    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.listener = tf.TransformListener()
        self.rate = Rate(5)
        self.shutdown_requested=False

    def callback(self, tag):
            if self.shutdown_requested:
                return
            if tag.detections:
                ttt=tag.detections[0]
                tag_id=ttt.id[0]
                try:
                    print("A")
                    if(tag_id==1):
                        self.listener.waitForTransform('/base_link', '/elvfront', rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.listener.lookupTransform('/base_link', '/elvfront', rospy.Time(0))
                    elif(tag_id==3):
                        print(tag_id)
                        self.listener.waitForTransform('/base_link', '/floor3', rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.listener.lookupTransform('/base_link', '/floor3', rospy.Time(0))
                    elif(tag_id==5):
                        print(tag_id)
                        self.listener.waitForTransform('/base_link', '/doorfront', rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.listener.lookupTransform('/base_link', '/doorfront', rospy.Time(0))
                    x = trans[0]
                    y = trans[1]
                    z = trans[2]
                    print(x)
                    if(tag_id==5):
                        if(x<1.5):
                            print("END!!!!!!!!")
                            self.shutdown_requested=True
                            return
                    angular = math.atan2(y, x)
                    linear = 0.20*math.sqrt(x ** 2 + y ** 2)

                    # # speed control
                    if linear<0.1:
                        linear=0.0
                    elif linear>0.18:
                        linear=linear*1.5
                    if angular<-0.0:
                        angular=angular*1.5
                    elif angular>0.0:
                        angular=angular*1.5

                    # velocity
                    cmd = Twist()
                    cmd.linear.x = linear
                    cmd.angular.z = angular
                    
                    try:
                        self.vel_pub.publish(cmd)
                    except KeyboardInterrupt:
                        print("Shutting down")

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Failed to lookup transform")
            else:
                print("NO DATA DETECTIONS!!")
            self.rate.sleep()
    def run(self):
        while not rospy.is_shutdown():
            if self.shutdown_requested:
                break
            rospy.sleep(0.1)
        rospy.signal_shutdown("Shutdown requested")


if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
       ic.run()
    except rospy.ROSInterruptException:
       print("Shutting down")