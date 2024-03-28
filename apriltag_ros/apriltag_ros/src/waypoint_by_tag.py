#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
import time
import tf

t1, t2, t3, t4, t5, t6, t7 = 0, 1, 2, 3, 4, 5, 6

class WaypointByTag:
    def __init__(self):
        self.shutdown_requested = False
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.tag_detected = [False, False, False, False, False, False, False] # tag 감지 여부
        self.waypoint_tf = TransformStamped()
        self.tag_tf = TransformStamped()
        self.listener = tf.TransformListener()
        self.distance_z = [0.645,-0.645, 0.645, 6.210, 5.850, 5.850, 0.750, 1.040, 0.000, 0.850, 0.850] # tag 기준 정면 방향 거리
        self.distance_x = [0.000, 0.000, 4.870, 0.000, 0.000,-0.700, 0.000, 0.275,-1.205, 0.000,-0.645] # tag 기준 오른쪽 방향 거리
        self.target_tag = [1, 2, 3, 4, 5, 6, 7]

    def callback(self, tag):
        if self.shutdown_requested:
            return

        if tag.detections and not self.tag_detected[t1]: # t1을 감지하지 못한 상태에서 tag를 감지한 경우
            for detection in tag.detections:
                if detection.id[0] == self.target_tag[t1]: # 감지된 tag 중 t1이 있는 경우
                    self.tag_detected[t1] = True # t1 감지 여부 = true

                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "/t1" # waypoint의 기준이 될 tag (tags.yaml 참고)
                    transform.child_frame_id = "/w1" # 생성할 waypoint 이름

                    transform.transform.translation.x = self.distance_x[0]
                    transform.transform.translation.y = 0.0
                    transform.transform.translation.z = self.distance_z[0]

                    transform.transform.rotation.x = 0.0
                    transform.transform.rotation.y = 0.0
                    transform.transform.rotation.z = 0.0
                    transform.transform.rotation.w = 1.0

                    self.tf_broadcaster.sendTransform(transform) # waypoint의 tf 생성

                    try:
                        self.listener.waitForTransform('/odom', '/w1', rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.listener.lookupTransform('/odom', '/w1', rospy.Time(0)) # odom과 waypoint 사이의 tf 관계 파악 후 저장

                        self.waypoint_tf.header.stamp = rospy.Time.now()
                        self.waypoint_tf.header.frame_id = "/odom"
                        self.waypoint_tf.child_frame_id = "/w1"

                        self.waypoint_tf.transform.translation.x = trans[0]
                        self.waypoint_tf.transform.translation.y = trans[1]
                        self.waypoint_tf.transform.translation.z = trans[2]

                        self.waypoint_tf.transform.rotation.x = rot[0]
                        self.waypoint_tf.transform.rotation.y = rot[1]
                        self.waypoint_tf.transform.rotation.z = rot[2]
                        self.waypoint_tf.transform.rotation.w = rot[3] # tag가 카메라에 잡히지 않을 경우를 고려하여 odom에 연결된 waypoint 정보 저장

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): # odom과 w1의 tf 관계를 얻지 못한 경우
                        rospy.logwarn("Failed to lookup transform")
            
    def loop(self):
        rate = rospy.Rate(10)  # 10Hz로 루프를 실행
        while not rospy.is_shutdown():
            if self.tag_detected[t1]: # t1 발견되었다면
                self.tf_broadcaster.sendTransform(self.waypoint_tf) # 저장해두었던 waypoint의 tf 정보 발행

                self.tag_tf.header.stamp = rospy.Time.now()
                self.tag_tf.header.frame_id = "/w1"
                self.tag_tf.child_frame_id = "/tt1"

                self.tag_tf.transform.translation.x = self.distance_x[1]
                self.tag_tf.transform.translation.y = 0.0
                self.tag_tf.transform.translation.z = self.distance_z[1]

                self.tag_tf.transform.rotation.x = 0.0
                self.tag_tf.transform.rotation.y = 0.0
                self.tag_tf.transform.rotation.z = 0.0
                self.tag_tf.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(self.tag_tf) # waypoint를 기준으로 임시 tag의 tf 정보 발행


            rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("waypoint_by_tag", anonymous=True)
    waypoint = WaypointByTag()
    waypoint.loop()
    rospy.spin()