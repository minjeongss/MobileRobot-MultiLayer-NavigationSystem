#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
import math
import tf2_geometry_msgs
import numpy as np
import subprocess

class cmd_pose:
    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.listener = tf.TransformListener()

        # 조건문 확인을 위한 변수
        self.reach_goal = [False for i in range(4)] # waypoint 도달 여부
        self.step = 0

        rospy.sleep(2)  # TransformListener가 변환 정보를 수집할 시간을 제공

    def move_to_wp(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(15.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        z = trans[2]
        angular = math.atan2(y, x)
        linear = 0.35 * math.sqrt(x ** 2 + y ** 2)

        # 위치
        if linear < 0.002:
            linear = 0.0
            self.reach_goal[self.step] = True
            rospy.loginfo("WAY POINT")
            rospy.sleep(1)
        elif linear > 0.18:
            linear = linear * 1.5

        #각도
        if angular < 0.0:
            angular = angular * 1.5
        elif angular > 0.0:
            angular = angular * 1.5

        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")

    def back_to_wp(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        z = trans[2]
        angular = math.atan2(y, x)
        linear = 0.35 * math.sqrt(x ** 2 + y ** 2)

        # 이동

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

    def spin(self, clockwise, radian):
        while (radian < 0): radian += 2*math.pi
        while (radian > 2*math.pi) : radian -= 2*math.pi
        self.listener.waitForTransform('/base_footprint', '/odom', rospy.Time(0), rospy.Duration(1.0))

        start_transform = TransformStamped()
        current_transform = TransformStamped()

        start_transform.transform = self.listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        
        base_cmd = Twist()
        base_cmd.linear.x = 0.0
        base_cmd.angular.z = 0.75

        if (clockwise): base_cmd.angular.z = -base_cmd.angular.z

        desired_turn_axis = np.array[0,0,1]
        if (not clockwise): desired_turn_axis = -desired_turn_axis

        rate = rospy.Rate(10.0)
        done = False
        while (not done):
            self.vel_pub.publish(base_cmd)
            rate.sleep()
            try:
                current_transform.transform = self.listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerror("Failed to lookup transform : %s", str(ex))
            relative_transform = tf2_geometry_msgs.do_transform_pose(current_transform, start_transform)
            rotated_quaternion = relative_transform.pose.orientation
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([rotated_quaternion.x, rotated_quaternion.y, rotated_quaternion.z, rotated_quaternion.w])
            actual_turn_axis = np.array[roll, pitch, yaw]
            angle_turned = math.sqrt(roll**2 + pitch**2 + yaw**2)
            if (math.fabs(angle_turned) < 1.0e-2): continue

            if (np.dot(actual_turn_axis, desired_turn_axis) < 0 ):
                angle_turned = 2*math.pi - angle_turned

            if (angle_turned > radian) : done = True
        self.reach_goal[self.step] = True

    def correction(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        z = trans[2]
        angular = math.atan2(y, x)
        linear = 0.0

        # 도착한 경우, 최종적으로 방향 조정
        
        if -0.01 < angular < 0.01:
            rospy.loginfo("FINISH")
            self.step = self.step + 1

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
                if not self.reach_goal[self.step]:
                    self.move_to_wp('/w1')
                elif self.reach_goal[self.step]:
                    self.correction('/t0')

            elif self.step == 1:
                if not self.reach_goal[self.step]:
                    self.spin(True, math.pi/2)
                elif self.reach_goal[self.step]:
                    self.step = self.step + 1

            elif self.step == 2:
                if not self.reach_goal[self.step]:
                    self.move_to_wp('/w2')
                elif self.reach_goal[self.step]:
                    self.correction('/t1')
                    
            elif self.step == 3:
                print("Finish!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                break

            rate.sleep()

if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
        ic.loop()
    except KeyboardInterrupt:
        print("Shutting down")
