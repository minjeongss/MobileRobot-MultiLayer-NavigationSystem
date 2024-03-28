#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
import math
import subprocess

class cmd_pose:
    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.listener = tf.TransformListener()

        # 조건문 확인을 위한 변수
        self.reach_goal = False # waypoint 도달 여부
        self.step = 0

        rospy.sleep(2)  # TransformListener가 변환 정보를 수집할 시간을 제공

    def back_to_wp(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        angular = math.atan2(y, x)
        linear = 0.35 * math.sqrt(x ** 2 + y ** 2)

        # 위치
        if linear > -0.002:
            linear = 0.0
            self.reach_goal[self.step] = True
            rospy.loginfo("WAY POINT")
            rospy.sleep(1)
        elif linear < -0.18:
            linear = linear * 1.5

        #각도
        if 0 < angular < math.pi:
            angular = (angular - math.pi) * 1.5
        elif 0 > angular > -math.pi:
            angular = (angular + math.pi) * 1.5

        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
         
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")

    def correction(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        angular = math.atan2(y, x)
        linear = 0.0

        # 도착한 경우, 최종적으로 방향 조정
        
        if -0.01 < angular < 0.01:
            rospy.loginfo("FINISH")
            self.step = self.step + 1
            if self.step == 1:
                subprocess.Popen("rosrun d435i_xarm_setup press_button_out")

        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")
        
    def loop(self):
        self.step
        rate = rospy.Rate(10)  # 10Hz로 루프를 실행합니다.
        while not rospy.is_shutdown():
            if self.step == 0:
                if not self.reach_goal:
                    self.back_to_wp('/w0')
                elif self.reach_goal:
                    self.correction('/t0')

            rate.sleep()

if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
        ic.loop()
    except KeyboardInterrupt:
        print("Shutting down")