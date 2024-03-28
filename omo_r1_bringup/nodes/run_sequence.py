#!/usr/bin/env python3
import time
import subprocess

def run_nodes(node_list, package_name):
    """노드 리스트와 패키지 이름을 받아 순서대로 실행하는 함수"""
    for node in node_list:
        subprocess.run(["rosrun", package_name, node])

# 첫 번째로 실행할 노드 리스트 및 패키지 이름
first_run_nodes = ["set_initial_pose.py", "movegoal1.py"]
first_run_package = "omo_r1_navigation"

# press_button_out 노드 정보
press_button_node = "press_button_out"
press_button_package = "d435i_xarm_setup"

# 두 번째로 실행할 노드 리스트 및 패키지 이름
second_run_nodes = ["movegoal2.py"]
second_run_package = "omo_r1_navigation"

# 세 번째로 실행할 노드 리스트 및 패키지 이름
third_run_nodes = ["turn_right.py", "detectNmove.py"]
third_run_package = "omo_r1_bringup"

# 첫 번째 노드 리스트 실행
run_nodes(first_run_nodes, first_run_package)

# press_button_out 노드 실행
time.sleep(3)  # 필요한 경우에 따라 대기 시간 조정
subprocess.run(["rosrun", press_button_package, press_button_node])

# 'rosservice call /move_base/clear_costmaps "{}"' 명령어 실행
subprocess.run(["rosservice", "call", "/move_base/clear_costmaps", "{}"])

# 두 번째 노드 리스트 실행
time.sleep(1)  # 필요한 경우에 따라 대기 시간 조정
run_nodes(second_run_nodes, second_run_package)

# 세 번째 노드 리스트 실행
run_nodes(third_run_nodes, third_run_package)

#time.sleep(1)
#run_nodes(third_run_nodes)

#time.sleep(3)  # 필요한    경우에 따라 대기 시간 조정
#subprocess.run(["rosrun", "d435i_xarm_setup", "press_button_in"])