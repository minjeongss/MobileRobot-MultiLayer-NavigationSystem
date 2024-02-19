#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from datetime import datetime

OMO_R1_MAX_LIN_VEL = 1.20
OMO_R1_MAX_ANG_VEL = 1.80

LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.3

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -OMO_R1_MAX_LIN_VEL, OMO_R1_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -OMO_R1_MAX_ANG_VEL, OMO_R1_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('omo_r1_teleop_elevator')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    # case for moving automatically without navigation
    param_value=rospy.get_param('~key_param','default_value')
    
    if param_value=='forward':
        first_time=datetime.now()

        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
        while(1):
            second_time=datetime.now()
            diff=second_time-first_time
            if(diff.seconds>3):
                break

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0

    if param_value=='backward':
        first_time=datetime.now()

        target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
        while(1):
            second_time=datetime.now()
            diff=second_time-first_time
            if(diff.seconds>2):
                break

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0

    if param_value=='halfspin':
        first_time=datetime.now()

        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
        while(1):
            second_time=datetime.now()
            diff=second_time-first_time
            print(diff.seconds)
            if(diff.seconds>8):
                break

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)