#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global last_time_printed

    # 현재 시간과 마지막으로 출력한 시간의 차이를 계산
    current_time = rospy.Time.now()
    if (current_time - last_time_printed).to_sec() < 1.0:
        # 1초가 지나지 않았다면 함수를 빠져나옴
        return

    # 여기서부터 데이터를 출력하고, 마지막으로 출력한 시간을 업데이트
    last_time_printed = current_time

    center_index = len(msg.ranges) // 2
    left_indices = range(center_index - 5, center_index)
    right_indices = range(center_index + 1, center_index + 6)

    print("왼쪽 5개 스캔값:")
    for i in left_indices:
        print(f"각도: {msg.angle_min + i * msg.angle_increment}, 거리: {msg.ranges[i]}m")

    print("오른쪽 5개 스캔값:")
    for i in right_indices:
        print(f"각도: {msg.angle_min + i * msg.angle_increment}, 거리: {msg.ranges[i]}m")
    print("\n")

if __name__ == '__main__':
    rospy.init_node('scan_values_node')
    last_time_printed = rospy.Time.now()  # rospy.init_node() 호출 이후로 이동
    scan_subscriber = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
