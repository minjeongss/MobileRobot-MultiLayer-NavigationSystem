#!/usr/bin/env python3
#객체인식 -> 네비게이션을 이용하여 omorobot 이동


import rospy
from geometry_msgs.msg import PoseStamped, Twist
from detection_msgs.msg import BoundingBoxes
import math
import tf

# Omorobot 이동 목표 설정 및 이동 클래스
class MoveToGoal:
    def __init__(self, x, y):
        rospy.init_node('move_to_goal_node', anonymous=True)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = x
        self.goal_msg.pose.position.y = y
        self.goal_msg.pose.orientation.w = 1.0
    
    def move_to_goal(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(self.goal_msg)
            rate.sleep()

# YOLO 객체 탐지 및 Omorobot 이동 클래스
class YoloOmoMover:
    def __init__(self):
        rospy.init_node('yolo_omo_mover', anonymous=True)
        self.bbox_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bbox_callback)
        self.listener = tf.TransformListener()

    def bbox_callback(self, data):
        if data.bounding_boxes:
            target_frame = "/4"  # 탐지된 객체의 프레임
            base_frame = "/base_link"

            try:
                self.listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(5.0))
                (trans, _) = self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
                goal_x = trans[0] - 0.1
                goal_y = trans[1] - 0.2

                mover = MoveToGoal(goal_x, goal_y)
                mover.move_to_goal()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform")

if __name__ == '__main__':
    try:
        YoloOmoMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
