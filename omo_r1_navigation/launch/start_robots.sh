#!/bin/bash

# omo_r1_robot_G2.launch 실행
roslaunch omo_r1_bringup omo_r1_robot_G2.launch
sleep 1

# omo_r1_navigation_G2.launch 실행
roslaunch omo_r1_navigation omo_r1_navigation_G2.launch
sleep 2

# omo_r1_navigation_rviz.launch 실행
roslaunch omo_r1_navigation omo_r1_navigation_rviz.launch
