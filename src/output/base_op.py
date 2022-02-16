#!/usr/bin/env python

import rospy
import math
import dynamic_reconfigure.client
from output.base_motion import BaseMotion
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from ICO.data_management import Data
from geometry_msgs.msg import Twist
from movo_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64, Bool, String
from vais.msg import vais_param

import time
import datetime

class Base_op(object):
    def __init__(self):

        #Instantiation
        self.move_cmd = Twist()
        self.output = BaseMotion()

        #Publishers
        self.motion_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1, latch=False)
        self.odom_capture_pub = rospy.Publisher('/signal/odom_capture',Bool, queue_size = 1)

        #Subscribers
        rospy.Subscriber('/signal/odom_capture',Bool, self.odom_capture_cb, queue_size = 1)
        rospy.Subscriber('/signal/shutdown', Bool, self.shutdown_cb, queue_size = 1)
        rospy.Subscriber('/robot/move', Bool, self.move_cb, queue_size = 1)
        rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odom_cb, queue_size = 1)
        rospy.Subscriber('/movo/feedback/active_configuration', Configuration, self.aconf_cb, queue_size=1)
        rospy.Subscriber('/data/vais_param', vais_param, self.vais_cb, queue_size=1)
        rospy.Subscriber('/ico/output', Float32, self.ico_cb, queue_size = 1)       


    #Reference position must be captured via an Odom capture signal.
    def ref_capture(self):
        if self.odom_capture == True:
            self.ref_odom = self.cur_odom[:]
            print("[INFO]: Reference Odometry is collected at: ", self.ref_odom)
            self.odom_capture=False
            #Signal back to make ref_odom unrewritable
            self.odom_capture_pub.publish(self.odom_capture)
        else:
            #print("[INFO]: Waiting for a reference signal")
            pass

    #Goal is generated   
    def goal(self, goal_odom):
        if self.ref_odom:
            #Once a reference is obtained, generates the target odom.
            self.target_odom(goal_odom)
            print("[INFO]: Target Odometry is generated at: ", self. tar_odom)
        else:
            pass


    #Target goal given by user
    def target_odom(self, goal_list):
        
        #Check turn direction
        if (goal_list[2] < 0):
            self.direction = "CW"
        else:
            self.direction = "CCW"

        #2D position and one yaw orientation
        pos_x = self.ref_odom[0]+goal_list[0]
        pos_y = self.ref_odom[1]+goal_list[1]
        orient_z = self.ref_odom[2]+goal_list[2]

        #Conversion from -180 to 180 into 0 to 360 degree range.
        if orient_z < 0 or orient_z > 360:
            orient_z = orient_z%360
        else:
            orient_z

        self.tar_odom = [pos_x, pos_y, orient_z]













    # Move by x meter(s)
    def forward(self, target, state, ico_out):

        #speed conversion
        speed = self.speed_conversion(ico_out, state)
    
        self.output.move_forward(target, speed)
        self.output.motion_stop()

        time.sleep(10)

    def backward(self, target, state, ico_out):

        #speed conversion
        speed = self.speed_conversion(ico_out, state)
    
        self.output.move_backward(target, speed)
        self.output.motion_stop()

        time.sleep(10)


    # + is a CCW rotaion, - is a CW rotation.
    def turn(self, target, state, ico_out):

        #speed conversion
        speed = self.speed_conversion(ico_out, state)

        if target>=0:
            self.output.rotate_clock(target, speed)
            self.output.motion_stop()
        else:
            self.output.rotate_anticlock(target,speed)
            self.output.motion_stop()
        time.sleep(10)

    def speed_conversion(self,weight, state):
        aconf = rospy.wait_for_message('/movo/feedback/active_configuration', Configuration)
        max_linear = aconf.x_vel_limit_mps
        max_angular = aconf.yaw_rate_limit_rps

        if state =="Linear":
            speed = max_linear -(max_linear*weight)
            return speed
        if state =="Angular":
            speed = max_angular - (max_angular*weight)
            return speed
