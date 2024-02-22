#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            print("YES")
            # 여기서 '/base_link'은 원하는 프레임이고, '/world'는 해당 프레임을 기준으로 하는 좌표계입니다.
            (trans, rot) = listener.lookupTransform('/map', '/floor3', rospy.Time(0))
            # trans는 위치, rot은 회전을 나타냅니다.
            print("Position:", trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("NO")
            continue

        rate.sleep()