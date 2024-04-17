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
                
                self.target_distance = current_distance - 1.5*self.initial_distance_1
                self.initial_distance_2 = current_distance
                self.initial_distance_1 = 0.001
            else:
                # 비례 제어를 사용한 속도 계산
                distance_difference = self.initial_distance_2 - current_distance
                goal_difference = self.target_distance - distance_difference

                if abs(goal_difference) < 0.004: 
                    linear = 0
                    self.publish_speed(linear)
                    rospy.sleep(1)
                    rospy.signal_shutdown("Target reached")
                
                else :
                    linear = 0.4 * goal_difference
                    if linear > 0.2 :
                        linear = 0.2
                    
                    if linear < 0.02 :
                        linear = 0.02
                    self.publish_speed(linear)
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
