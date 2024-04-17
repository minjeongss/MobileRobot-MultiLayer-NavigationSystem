#!/usr/bin/env python3
# 라이다 보고 뒤로 이동하는 코드 

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserBackMover:
    def __init__(self):
        rospy.init_node('laser_back_mover', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.initial_distance = None
        self.target_back_distance = 0.3  # 30cm 뒤로 이동 목표

    def scan_callback(self, msg):
        center_index = len(msg.ranges) // 2
        indices = range(center_index - 4, center_index + 4)
        ranges = [msg.ranges[i] for i in indices if msg.ranges[i] > 0]

        if not ranges:
            return

        current_distance = sum(ranges) / len(ranges)

        if self.initial_distance is None:
            self.initial_distance = current_distance
            rospy.loginfo(f"Initial distance set to: {self.initial_distance}m")

        # 목표로부터의 현재 거리 차이 계산
        distance_difference = current_distance - self.initial_distance
        goal_difference = distance_difference - self.target_back_distance

        rospy.loginfo(f"Distance to target: {distance_difference - self.target_back_distance}m")

        # 거리 차이가 절대값으로 0.01보다 작거나 같으면 목표 도달로 간주
        if abs(goal_difference) <= 0.004:
            rospy.loginfo("Target reached. Stopping the robot.")
            self.publish_speed(0)
            rospy.signal_shutdown("Target reached")
        else:
            # 뒤로 이동하는 비례 제어 속도 계산
            linear = 0.4 * goal_difference
            if linear < -0.2:
                linear = -0.2

            if linear > -0.02 and linear <=0 :
                linear = -0.02
            self.publish_speed(linear)

    def publish_speed(self, linear):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        self.vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        LaserBackMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Laser Back Mover node shutdown.")
