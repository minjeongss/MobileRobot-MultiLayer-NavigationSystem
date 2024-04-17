#!/usr/bin/env python3
# YOLO로 자세 정렬하기 

import rospy
import tf
from geometry_msgs.msg import Twist
import math
from detection_msgs.msg import BoundingBoxes
import time

class YoloAligner:
    def __init__(self):
        rospy.init_node('yolo_aligner', anonymous=True)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.bbox_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bbox_callback)
        self.listener = tf.TransformListener()

    def bbox_callback(self, data):
        
        if data.bounding_boxes:

            target_bbox = data.bounding_boxes[0]
            target_frame = "/4"  # This should be the frame associated with the detected object
            base_frame = "/base_link"

            try:
                self.listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
                x = trans[0]
                y = trans[1]
                z = trans[2]

                target_x = 0.58
                #target_y = -0.15

                #target_distance = math.sqrt(target_x **2 + target_y **2)
                #target_angular = math.atan2(target_y, target_x)

                current_distance = x
                #current_angular = math.atan2(y,x)
                

                # 각도 조정을 위한 비례 제어
                diff_distance = current_distance - target_x
                #diff_angular = current_angular - target_angular

                linear = 0.2 * diff_distance  # 선속도 조정
                #angular = 0.4 * diff_angular  # 각속도 조정

                if linear > 0.15:
                    linear = 0.15
                #if angular > 0.3:
                #    angular = 0.3

                # 속도 조정
                if abs(diff_distance) < 0.03:
                    linear = 0
                #if abs(diff_angular) < 0.00001:
                #    angular = 0

                # 속도 명령 전송
                twist = Twist()
                #twist.angular.z = angular
                twist.linear.x = linear
                self.vel_pub.publish(twist)

                # 목표 도달 여부 확인
                if linear == 0:
                    rospy.sleep(1)
                    rospy.signal_shutdown("Target reached")
                    print("Target reached, shutting down node")

                self.last_time = time.time()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform")
        else:
            print("NO DATA DETECTIONS!!")
            linear = 0
            angular = 0 
            twist = Twist()
            twist.angular.z = angular
            twist.linear.x = linear
            self.vel_pub.publish(twist)            

if __name__ == '__main__':
    try:
        yolo_aligner = YoloAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")
