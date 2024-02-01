#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

global robot_pose_x
global robot_pose_y
robot_pose_x=0
robot_pose_y=0

def pose_callback(msg):
    global robot_pose_x
    global robot_pose_y

    #print("def: ",msg.pose.pose.position.x)
    robot_pose_x=msg.pose.pose.position.x
    robot_pose_y=msg.pose.pose.position.y

def move_to_goal(x, y):
    # ROS 노드 초기화
    rospy.init_node('move_to_goal_node', anonymous=True)

    # Publisher, Subscriber 생성
    # 목표 설정하는 topic 부분
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    # 이동 목표 생성
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = 'map'  # 이동 기준 좌표계 설정
    goal_msg.pose.position.x = x
    goal_msg.pose.orientation.z = 0.9063
    goal_msg.pose.orientation.w = 0.4226  # 머리 방향은 기본적으로 정면으로 향하도록 설정
    print("[1] setup goal")

    # 일정 주기로 메시지 발행
    rate = rospy.Rate(5)  # 5Hz로 설정, 20Hz는 불가능, 기존 설정은 10Hz
    while not rospy.is_shutdown():
        global robot_pose_x
        global robot_pose_y
        print("[2] setup robot pose")
        # 로봇 위치 topic 부분
        odom_sub=rospy.Subscriber('/odom',Odometry,pose_callback)
        print("Odom: ",odom_sub) #실제 값이 아님에 유의!

        print("[3] calculate distance")
        print("rx: ",robot_pose_x,"x: ",x,"ry: ",robot_pose_y,"y: ",y)
        print("rx-x: ",robot_pose_x-x,"ry-y: ",robot_pose_y-y)
        distance=math.sqrt((robot_pose_x-x)**2+(robot_pose_y-y)**2)
        print("distance:", distance)
        if(distance<0.1):
            print("succeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeed")

        pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 이동할 좌표 설정: 턱 전까지
        goal_x = -5.45528
        goal_y = 1.38668

        # 목표 지점으로 이동 명령 발행
        move_to_goal(goal_x, goal_y)

    except rospy.ROSInterruptException:
        pass