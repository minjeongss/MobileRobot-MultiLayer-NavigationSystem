#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid


class AprilTagUpdateNode:
    def __init__(self):
        rospy.init_node('apriltag_update_node', anonymous=True)

        # AprilTag 정보를 받는 토픽 구독(def apriltag_callback)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.apriltag_callback)
        
        # make tppic part
        # tf 변환
        self.tf_broadcaster = TransformBroadcaster()
        # Rviz에 AprilTag을 시각화
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # 맵 데이터 초기화
        self.map_data = OccupancyGrid()

    def apriltag_callback(self, data):
        for tag in data.detections:
            print("[Check] Received Apriltag data: ",data)
            tag_id = tag.id[0]
            tag_pose = tag.pose.pose

            # function call
            # TF 변환을 생성하여 브로드캐스트
            self.broadcast_apriltag_tf(tag_pose, "/map", f"/apriltag_{tag_id}")

            # AprilTag을 RViz에서 시각화
            self.visualize_apriltag(tag_pose, tag_id)

    def broadcast_apriltag_tf(self, tag_pose, parent_frame, child_frame):
        translation = (tag_pose.position.x, tag_pose.position.y, tag_pose.position.z)
        rotation = (
            tag_pose.orientation.x,
            tag_pose.orientation.y,
            tag_pose.orientation.z,
            tag_pose.orientation.w
        )

        current_time = rospy.Time.now()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = translation
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = rotation

        self.tf_broadcaster.sendTransform(tf_msg)

    def visualize_apriltag(self, tag_pose, tag_id):

        # 로봇의 위치를 맵 상의 좌표로 변환
        robot_x = int((tag_pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        robot_y = int((tag_pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # AprilTag의 위치를 맵에 반영 (예: 특정 값을 마킹)
        if 0 <= robot_x < self.map_data.info.width and 0 <= robot_y < self.map_data.info.height:
            self.map_data.data[robot_y * self.map_data.info.width + robot_x] = 50  # Set cell as marked (50)

        # 업데이트된 맵을 다시 발행
        self.map_data.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_data)

        # RViz에 AprilTag을 시각화하는 Marker 메시지 생성
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.id = tag_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = tag_pose

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        print("[Check] Publishing marker: ",marker)
        self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = AprilTagUpdateNode()
    node.run()