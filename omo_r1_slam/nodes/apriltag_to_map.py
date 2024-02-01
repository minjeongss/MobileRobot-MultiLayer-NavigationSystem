#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
import math
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class AprilTagUpdateNode:
    def __init__(self):
        rospy.init_node('apriltag_update_node', anonymous=True)

        # AprilTag 정보를 받는 토픽 구독
        # def apriltag_callback
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.apriltag_callback)

        # 로봇 위치 업데이트를 위한 토픽 발행
        # make topic part
        self.pose_pub = rospy.Publisher('robot_pose_update', PoseWithCovarianceStamped, queue_size=10)

        # SLAM 패키지로부터 로봇의 현재 위치를 받아오는 토픽 구독
        # def robot_pos_callback
        rospy.Subscriber('/robot_pose', PoseStamped, self.robot_pose_callback)
        # 맵 정보를 받는 토픽 구독
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Rviz에 AprilTag을 시각화하기 위한 퍼블리셔
        # make topic part
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # 맵 데이터를 저장할 변수
        self.map_data = None 

    def apriltag_callback(self, data):
        if data.detections:
            # 1. apriltag detection+publish robot localozation
            # 첫 번째로 감지된 AprilTag의 위치와 ID 사용 (다양한 로직을 적용할 수 있음)
            print("[Check] Received Apriltag data: ",data)
            tag = data.detections[0]
            tag_id = tag.id[0]
            tag_pose = tag.pose.pose

            # I dont want to translate apriltag to robot pose!
            # # 로봇의 위치 업데이트를 위해 PoseWithCovarianceStamped 메시지 생성
            # pose_msg = PoseWithCovarianceStamped()
            # pose_msg.header.stamp = rospy.Time.now()
            # pose_msg.header.frame_id = 'map'  # 현재는 map frame으로 가정

            # # AprilTag의 위치를 로봇의 위치로 설정
            # pose_msg.pose.pose.position.x = tag_pose.position.x
            # pose_msg.pose.pose.position.y = tag_pose.position.y
            # pose_msg.pose.pose.position.z = tag_pose.position.z

            # # Quaternion을 Euler 각도로 변환
            # (roll, pitch, yaw) = euler_from_quaternion([
            #     tag_pose.orientation.x,
            #     tag_pose.orientation.y,
            #     tag_pose.orientation.z,
            #     tag_pose.orientation.w
            # ])

            # # Yaw 각도를 조정하거나 필요한 로직을 추가
            # # 예: yaw += math.pi / 2.0

            # # Euler 각도를 Quaternion으로 변환
            # q = quaternion_from_euler(roll, pitch, yaw)

            # # 로봇의 방향 업데이트
            # pose_msg.pose.pose.orientation.x = q[0]
            # pose_msg.pose.pose.orientation.y = q[1]
            # pose_msg.pose.pose.orientation.z = q[2]
            # pose_msg.pose.pose.orientation.w = q[3]

            # # 업데이트된 로봇의 위치 정보를 발행
            # # publish topic part
            # print("[Check] Publishing robot pose: ",pose_msg)
            # self.pose_pub.publish(pose_msg)

            #2. shwo april tag to rviz
            # Rviz에 AprilTag을 시각화하는 Marker 메시지 생성
            marker = Marker()
            marker.header.frame_id = 'map'  # 현재는 map frame으로 가정
            marker.header.stamp = rospy.Time.now()
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position = tag_pose.position
            marker.pose.orientation = tag_pose.orientation

            marker.scale.x = 0.2  # Marker 크기 조절
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Rviz에 Marker 메시지를 퍼블리시하여 시각화
            # publish topic part
            print("[Check] Publishing marker: ",marker)
            self.marker_pub.publish(marker)
    
    def robot_pose_callback(self, data):
        # SLAM 패키지로부터 받은 로봇의 현재 위치 정보
        robot_pose = data.pose

        # 로봇의 위치를 맵 상의 좌표로 변환
        robot_x = int((robot_pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        robot_y = int((robot_pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # AprilTag의 위치를 맵에 반영 (예: 특정 값을 마킹)
        if 0 <= robot_x < self.map_data.info.width and 0 <= robot_y < self.map_data.info.height:
            self.map_data.data[robot_y * self.map_data.info.width + robot_x] = 50  # Set cell as marked (50)

        # 업데이트된 맵을 다시 발행
        self.map_data.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_data)

        # extra part
        # 로봇의 위치 업데이트 관련 코드 추가 (예: 보정, 필요에 따라 추가 로직)

        # 업데이트된 로봇의 위치 정보를 활용하여 AprilTag의 위치 업데이트
        # ... (AprilTag 업데이트 관련 코드)

    def map_callback(self, data):
        # 맵 데이터를 받으면 저장
        self.map_data = data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = AprilTagUpdateNode()
    node.run()
