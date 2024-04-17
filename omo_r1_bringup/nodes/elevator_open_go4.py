#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserForwardMover:
    def __init__(self):
        rospy.init_node('laser_forward_mover', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.initial_distance_1 = None
        self.initial_distance_2 = None
        self.target_distance = None
        self.ignore_count = 0  # 새로운 카운터 변수

    def scan_callback(self, msg):
        center_index = len(msg.ranges) // 2
        indices = range(center_index - 2, center_index + 2)
        ranges = [msg.ranges[i] for i in indices if msg.ranges[i] > 0]

        if not ranges:
            return

        current_distance = sum(ranges) / len(ranges)

        if self.initial_distance_1 is None:
            self.initial_distance_1 = current_distance
            rospy.loginfo(f"Initial distance set to: {self.initial_distance_1}m")
            return

        if current_distance >= 2 * self.initial_distance_1:
            if self.target_distance is None:
                if self.ignore_count < 5:  # 3번의 측정값을 버립니다.
                    self.ignore_count += 1
                    return
                
            self.publish_speed(0.2)  # 앞으로 전진 속도 설정
            rospy.sleep(4)  # 4초 동안 대기
            self.publish_speed(0)  # 정지
            rospy.signal_shutdown("Target reached and moved for 4 seconds.")  # 노드 종료
            
                
        else:
            rospy.loginfo("Waiting for the door to open...")

    def publish_speed(self, linear):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        self.vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        LaserForwardMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Laser Forward Mover node shutdown.")
