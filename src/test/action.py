#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from output.base_output import MOVO_output
import time

class TEST_Action(object):
    def __init__(self):

        #Instantiation
        self.move_cmd = Twist()
        #publisher(s)
        self.motion_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1, latch=False)
        #subscriber(s)
        rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odom_cb, queue_size = 1)

    #Main action
    def main(self):
        print('main')
        self.action1(0.5)
        self.action2(-0.5)


    def action1(self, input):
        print('action1')
        #self.flag = False
        for x in range(100):
            time.sleep(0.05)
            print('act1',x)
            self.motion_pub.publish(self.twist_body(0,0,input))
        #self.flag = True

    def action2(self, input):
        #self.flag = False
        print('action2')
        for x in range(100):
            time.sleep(0.05)
            print('act2',x)
            self.motion_pub.publish(self.twist_body(0,0,input))
        #self.flag = True

    #Speed message constructor
    def twist_body(self, linear_x, linear_y, angular_z):
        #Axis X/Y, linear move
        self.move_cmd.linear.x = linear_x
        self.move_cmd.linear.y = linear_y
        #Axis Z, angular turn
        self.move_cmd.angular.z = angular_z
        return self.move_cmd

            #Wheel Odometry callback
    def odom_cb(self, value):
        print('odom')
        pos_x = value.pose.pose.position.x
        pos_y = value.pose.pose.position.y
        orient_z = self.quaternion_to_euler(value.pose.pose.orientation.x, value.pose.pose.orientation.y, value.pose.pose.orientation.z, value.pose.pose.orientation.w)
        self.cur_odom = [pos_x, pos_y, orient_z]

        self.main()

    #angle conversion
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        #Service robot doesn't have roll and pitch rotation, Only yaw is active
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        #Convert into a range of 0 to 360
        if Z < 0:
            return Z%360
        else:
            return Z
    