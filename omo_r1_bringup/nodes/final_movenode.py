#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes
import math
import time
import subprocess

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.bbox_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bbox_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)

        self.goal_x = 0.6
        self.goal_y = - 0.3
        self.x = None
        self.y = None
        self.initial_distance_x = None
        self.initial_distance_y = None
        self.current_stage = 0
        self.detected = False
        self.scan_data_ready = False  # 새로운 변수 추가

        self.initial_distance = None #5단계에서 사용

        self.initial_distance_1 = None
        self.initial_distance_2 = None
        self.target_distance = None
        self.ignore_count = 0  # 엘리베이터 문 열리면 전진하는 변수


    def bbox_callback(self, data):
        if self.current_stage == 2:
            self.turn_left_80_degrees()

        if self.current_stage == 4:
            self.execute_command()
            self.scan_data_ready = True
            self.current_stage = 5
            #rospy.signal_shutdown("Target reached") 
        
        if self.current_stage == 6:
            self.turn_right_80_degrees()

        if self.detected:
            return
        
        if data.bounding_boxes:
            target_bbox = data.bounding_boxes[0]  
            target_frame = "/2"  
            base_frame = "/base_link"
            try:
                self.listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
                self.x = trans[0]
                self.y = trans[1]
                z = trans[2]
                self.target_front_distance = self.x - self.goal_x                
                self.target_back_distance = self.goal_y - self.y
                rospy.loginfo("self.x: {}".format(self.y))
                rospy.loginfo("self.target_back_distance: {}".format(self.target_back_distance))

                self.detected = True
                self.scan_data_ready = True
                self.current_stage = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF Exception: %s" % str(e))

    def scan_callback(self, msg):
        if not self.scan_data_ready:
            return  # scan_data_ready가 False이면 처리하지 않음
        if self.current_stage == 1:

            center_index = len(msg.ranges) // 2
            indices = range(center_index - 4, center_index + 4)
            ranges = [msg.ranges[i] for i in indices if msg.ranges[i] > 0]

            if not ranges:
                return

            current_distance_x = sum(ranges) / len(ranges)

            if self.initial_distance_x is None:
                self.initial_distance_x = current_distance_x
                rospy.loginfo(f"Initial distance set to: {self.initial_distance_x}m")

            
            distance_difference_x = self.initial_distance_x - current_distance_x
            goal_difference_x = self.target_front_distance - distance_difference_x
            rospy.loginfo(f"Distance to target: {self.target_front_distance - distance_difference_x}m")

            if abs(goal_difference_x) <= 0.004:
                linear_x = 0
                self.publish_speed(linear_x)
                rospy.sleep(1)
                self.scan_data_ready = False
                self.current_stage = 2          
            else:
                # 앞으로로 이동하는 비례 제어 속도 계산
                linear_x = 0.4 * goal_difference_x
                if linear_x > 0.2 :
                    linear_x = 0.2

                if linear_x <0.05 :
                    linear_x = 0.05
                self.publish_speed(linear_x)  
                
        elif self.current_stage == 3:
            
            center_index = len(msg.ranges) // 2
            indices = range(center_index - 4, center_index + 4)
            ranges = [msg.ranges[i] for i in indices if msg.ranges[i] > 0]      

            if not ranges:
                return

            current_distance_y = sum(ranges) / len(ranges)

            if self.initial_distance_y is None:
                self.initial_distance_y = current_distance_y
                rospy.loginfo(f"Initial distance set to: {self.initial_distance_y}m")

            
            distance_difference_y = current_distance_y - self.initial_distance_y
            goal_difference_y = distance_difference_y - self.target_back_distance
            rospy.loginfo(f"Initial : {self.initial_distance_y}m")
            rospy.loginfo("current : {}".format(current_distance_y))


            ##rospy.loginfo(f"Distance to target y: {distance_difference - self.target_back_distance}m")

            if abs(goal_difference_y) <= 0.004:
                linear_y = 0
                self.publish_speed(linear_y)
                rospy.sleep(1)
                self.scan_data_ready = False
                self.current_stage = 4
                #rospy.signal_shutdown("Target reached")     
            else:
                # 뒤로로 이동하는 비례 제어 속도 계산
                linear_y = 0.4 * goal_difference_y
                if linear_y < - 0.2 :
                    linear_y = - 0.2

                if linear_y >-0.02 and linear_y <= 0 :
                    linear_y = -0.02

                self.publish_speed(linear_y)     
            
        elif self.current_stage == 5:
            
            center_index = len(msg.ranges) // 2
            indices = range(center_index - 4, center_index + 4)
            ranges = [msg.ranges[i] for i in indices if msg.ranges[i] > 0]      

            if not ranges:
                return

            current_distance = sum(ranges) / len(ranges)

            if self.initial_distance is None:
                self.initial_distance = current_distance
                rospy.loginfo(f"Initial distance set to: {self.initial_distance}m")

            
            distance_difference = self.initial_distance - current_distance
            goal_difference = self.target_back_distance+0.03 - distance_difference

            ##rospy.loginfo(f"Distance to target y: {distance_difference - self.target_back_distance}m")

            if abs(goal_difference) <= 0.004:
                linear = 0
                self.publish_speed(linear)
                rospy.sleep(1)
                self.current_stage = 6 
                self.scan_data_ready = False

 
            else:
                linear = 0.4 * goal_difference
                if linear > 0.2 :
                    linear = 0.2

                if linear < 0.05 :
                    linear = 0.05
                self.publish_speed(linear)  
                   
        elif self.current_stage == 7:
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

            if abs(current_distance-self.initial_distance_1) >= 0.02:
                
                self.stage_seven_moving_forward()
                rospy.signal_shutdown("Target reached and moved for 4 seconds.")  # 노드 종료
            
                
        else:
            rospy.loginfo("Waiting for the door to open...")
    

    def turn_left_80_degrees(self):
            
            turn_msg = Twist()
            turn_msg.angular.z = math.radians(45)  # Set rotation speed to 45 degrees per second

            # Calculate the time required to complete a 90 degree turn
            turn_time = math.radians(77) / turn_msg.angular.z

            # Start turning
            start_time = time.time()
            while time.time() - start_time < turn_time:
                self.vel_pub.publish(turn_msg)
                self.rate.sleep()
                
            turn_msg.angular.z = 0
            self.vel_pub.publish(turn_msg)

            rospy.sleep(1)
            # Stop the robot after turning
            self.scan_data_ready = True
            self.current_stage = 3

    def turn_right_80_degrees(self):
            turn_msg = Twist()
            turn_msg.angular.z = - math.radians(45)  # Set rotation speed to 45 degrees per second

            # Calculate the time required to complete a 90 degree turn
            turn_duration = math.radians(77) / abs(turn_msg.angular.z)
            end_time = rospy.Time.now() + rospy.Duration(turn_duration)
            # Start turning
            while rospy.Time.now() < end_time:
                self.vel_pub.publish(turn_msg)
                self.rate.sleep()
                
            turn_msg.angular.z = 0
            self.vel_pub.publish(turn_msg)

            rospy.sleep(1)
            # Stop the robot after turning
            self.scan_data_ready = True
            self.current_stage = 7

    def stage_seven_moving_forward(self):
        forward_msg = Twist()
        forward_msg.linear.x = 0.2  # 전진 속도 설정
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 7:  # 4초 동안 계속 명령 보내기
            self.vel_pub.publish(forward_msg)
            self.rate.sleep()  # 다른 콜백이 차단되지 않도록 루프 동안 일시 중지

        forward_msg.linear.x = 0
        self.vel_pub.publish(forward_msg)
        rospy.signal_shutdown("목표 도달 및 4초 동안 이동 완료.")  # 노드 종료
           

    def publish_speed(self, linear):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        self.vel_pub.publish(twist_msg)

    def publish_angular_speed(self, angular_speed):
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed
        self.vel_pub.publish(twist_msg)

    def execute_command(self):
        try:
            subprocess.run(['rosrun', 'd435i_xarm_setup', 'press_button_out'], check=True)
            rospy.loginfo("Command executed successfully.")
        except subprocess.CalledProcessError:
            rospy.logerr("Failed to execute command.")                    
        
if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Controller node shutdown.")
