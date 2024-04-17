#!/usr/bin/env python3
#라이다 값으로 오모로봇 자세 정렬하기

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserScanAligner:
    def __init__(self):
        rospy.init_node('laser_scan_aligner')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def scan_callback(self, msg):
        center_index = len(msg.ranges) // 2
        left_indices = range(center_index - 5, center_index)
        right_indices = range(center_index + 1, center_index + 6)

        # 왼쪽과 오른쪽 스캔 값 중 유효한 값만 필터링 (0 이상)
        left_ranges = [msg.ranges[i] for i in left_indices if msg.ranges[i] > 0]
        right_ranges = [msg.ranges[i] for i in right_indices if msg.ranges[i] > 0]

        # 왼쪽과 오른쪽 스캔값의 평균 거리 계산
        left_distance = sum(left_ranges) / len(left_ranges) if left_ranges else 0
        right_distance = sum(right_ranges) / len(right_ranges) if right_ranges else 0

        distance_diff = left_distance - right_distance

        if abs(distance_diff) <= 0.000004:
            # 거리 차이가 -0.03과 0.03 사이이면, 회전하지 않음
            angular_speed = 0.0
            # 속도 메시지 발행 후 노드 종료
            self.publish_speed(angular_speed)
            rospy.signal_shutdown("Alignment Complete")
        else:
            # 조건에 따라 적절한 방향으로 회전
            angular_speed = -0.04 if distance_diff < 0 else 0.04
            
            # 속도 메시지 생성 및 발행
            self.publish_speed(angular_speed)

    def publish_speed(self, angular_speed):
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed
        self.vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        LaserScanAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
