#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import json
import subprocess

class MoveToGoal:
    def __init__(self, x, y):
        rospy.init_node('move_to_goal_node', anonymous=True)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)  # cmd_vel 토픽 구독
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'  # 이동 기준 좌표계 설정
        self.goal_msg.pose.position.x = x
        self.goal_msg.pose.position.y = y
        self.goal_msg.pose.orientation.w = 1.0  # 머리 방향은 기본적으로 정면으로 향하도록 설정
        self.flag=0
    def move_to_goal(self):
        rate = rospy.Rate(5)  # 5Hz로 설정, 20Hz는 불가능, 기존 설정은 10Hz
        while not rospy.is_shutdown():
            self.pub.publish(self.goal_msg)
            rate.sleep()
    def cmd_vel_callback(self, msg):
        # cmd_vel 값을 받아와서 속도가 0인지 확인하여 종료
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.flag+=1
            if self.flag==10:
                rospy.signal_shutdown("Robot reached goal")
                rospy.sleep(1) 
if __name__ == '__main__':
    try:
        goal_x = 7.70566
        goal_y = -8.42377
        # 목표 지점으로 이동 명령 발행
        mover = MoveToGoal(goal_x, goal_y)
        mover.move_to_goal()
    except rospy.ROSInterruptException:
        pass