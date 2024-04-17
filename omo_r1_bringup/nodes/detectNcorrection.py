#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
import math

class cmd_pose:
    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)                     #속도, 각속도 값 publish
        self.listener = tf.TransformListener()

        # 조건문 확인을 위한 변수
        self.reach_goal = [False, False, False, False, False, False, False, False, False, False, False, False] # waypoint 도달 여부
        self.step = 0

        rospy.sleep(2)  # TransformListener가 변환 정보를 수집할 시간을 제공

    def move_to_wp(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))  #tf가 될 때까지 대기
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))         #

        x = trans[0]
        y = trans[1]
        z = trans[2]
        angular = math.atan2(y, x)
        linear = 0.35 * math.sqrt(x ** 2 + y ** 2)

        # 이동

        # 위치
        if linear < 0.002:
            linear = 0.0
            self.reach_goal[self.step] = True
            rospy.loginfo("WAY POINT")
            rospy.sleep(2)
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
            rospy.sleep(2)
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

    def spin_to_tag(self, goal):
        self.listener.waitForTransform('/base_link', goal, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('/base_link', goal, rospy.Time(0))

        x = trans[0]
        y = trans[1]
        z = trans[2]
        angular = math.atan2(y, x)
        linear = 0.0

        # waypoint에 도달했다면
 
        if -0.1 < angular < 0.1:
            self.reach_goal[self.step] = True
            rospy.loginfo("TAG")
            rospy.sleep(2)

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
            try:
                if self.step == 0:
                    if not self.reach_goal[self.step]:
                        self.move_to_wp('/w1')
                    elif self.reach_goal[self.step]:
                        self.step = self.step + 1

                elif self.step == 1:
                    if not self.reach_goal[self.step]:
                        self.spin_to_tag('/tt1')
                    elif self.reach_goal[self.step]:
                        self.correction('/t1')

                elif self.step == 2:
                    if not self.reach_goal[self.step]:
                        self.spin_to_tag('/tt2')
                    elif self.reach_goal[self.step]:
                        self.correction('/t2')

                elif self.step == 3:
                    if not self.reach_goal[self.step]:
                        self.back_to_wp('/w2')
                    elif self.reach_goal[self.step]:
                        self.correction('/t2')

                elif self.step == 4:
                    if not self.reach_goal[self.step]:
                        self.move_to_wp('/w3')
                    elif self.reach_goal[self.step]:
                        self.correction('/t2')

                elif self.step == 5:
                    if not self.reach_goal[self.step]:
                        self.spin_to_tag('/tt3')
                    elif self.reach_goal[self.step]:
                        self.correction('/ttt3')

                elif self.step == 6:
                    if not self.reach_goal[self.step]:
                        self.move_to_wp('/w4')
                    elif self.reach_goal[self.step]:
                        self.correction('/t4')

                elif self.step == 7:
                    if not self.reach_goal[self.step]:
                        self.back_to_wp('/w5')
                    elif self.reach_goal[self.step]:
                        self.step = self.step + 1

                elif self.step == 8:
                    if not self.reach_goal[self.step]:
                        self.spin_to_tag('/tt5')
                    elif self.reach_goal[self.step]:
                        self.correction('/t5')

                elif self.step == 9:
                    if not self.reach_goal[self.step]:
                        self.move_to_wp('/w6')
                    elif self.reach_goal[self.step]:
                        self.correction('/t5')

                elif self.step == 10:
                    if not self.reach_goal[self.step]:
                        self.spin_to_tag('/tt6')
                    elif self.reach_goal[self.step]:
                        self.correction('/ttt6')
                        
                # step 11 : 밖으로 나가기
                    

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform")
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
        ic.loop()
    except KeyboardInterrupt:
        print("Shutting down")








