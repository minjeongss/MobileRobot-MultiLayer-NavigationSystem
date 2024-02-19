#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import subprocess

global flag
flag=-1

class ApriltagDetector:
    def __init__(self):
        global flag
        flag=0
        rospy.init_node('apriltag_detector')
        self.tag_sub=rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_callback)
        tag=[]

    def list_running_nodes(self):
        result = subprocess.check_output(['rosnode', 'list'])
        running_nodes = result.decode('utf-8').split('\n')
        return [node for node in running_nodes if node]

    def kill_ros_node(self, node_name):
        try:
            # 'rosnode info' 명령을 사용하여 노드 정보를 얻음
            node_info = subprocess.check_output(['rosnode', 'info', node_name]).decode('utf-8')

            # 노드의 PID를 찾음
            pid_line = [line for line in node_info.split('\n') if 'Pid' in line]
            
            if not pid_line:
                rospy.logwarn(f"Cannot find PID for node '{node_name}'. It may not be running.")
                return

            pid = int(pid_line[0].split(':')[1].strip())

            # PID를 사용하여 노드를 종료
            subprocess.check_call(['kill', str(pid)])
            
            rospy.loginfo(f"Node '{node_name}' killed successfully.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to kill node '{node_name}': {e}")
        except Exception as ex:
            rospy.logerr(f"An error occurred: {ex}")

    def tag_callback(self,data):
        global flag
        if data.detections:
            # AprilTag 정보 받아오는 부분
            tag=data.detections[0]
            tag_id=tag.id[0]
            tag_x=tag.pose.pose.pose.position.x
            tag_y=tag.pose.pose.pose.position.y
            print("ID: ",tag_id)
            #print("x: ",tag_x,"y: ",tag_y)
            
            # AprilTag 인식해 multi floor 구현
            if(tag_id==0):
                if flag==0:
                    self.kill_ros_node('/robot_state_publisher')
                    self.kill_ros_node('/map_server')
                    roslaunch_command = "roslaunch omo_r1_navigation omo_r1_navigation_G2.launch map_file:='/home/minjeong/catkin_ws/src/omo_r1_navigation/maps/map3f.yaml'"
                    try:
                    	 #subprocess.Popen("rosrun robot_state_publisher robot_state_publisher", shell=True)
                        subprocess.Popen(roslaunch_command, shell=True)
                        flag=1
                    except Exception as e:
                        print(f"Error while running roslaunch: {e}")
                print("Floor 0!!!!!!!")
                tag=[]
            elif(tag_id==1):
                if flag==1:
                    self.kill_ros_node('/map_server')
                    roslaunch_command = "roslaunch omo_r1_navigation omo_r1_navigation_G2.launch map_file:='/home/minjeong/catkin_ws/src/omo_r1_navigation/maps/map.yaml'"
                    try:
                        subprocess.Popen(roslaunch_command, shell=True)
                        flag=2
                    except Exception as e:
                        print(f"Error while running roslaunch: {e}")
                print("Floor 1!")
                tag=[]

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    detector=ApriltagDetector()

    # /map_server 찾는 용도
    # running_nodes = detector.list_running_nodes()
    # print("Running nodes:")
    # for node in running_nodes:
    #     print(node)

    detector.run()
