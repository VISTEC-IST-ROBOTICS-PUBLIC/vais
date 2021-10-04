#!/usr/bin/env python

#MOVO modules
#from system_defines import *
#Note use rqt_reconfigure to modify maximum velocity/acceleration of MOVO to 1.0m/s and 0.5m/s2

from movo_msgs.msg import *
from geometry_msgs.msg import Twist
from vais.msg import vais_param
from std_msgs.msg import Float32, Float64, Bool, String
from nav_msgs.msg import Odometry
import rospy
import math
import numpy as np

class demo3(object):
    def __init__(self):

        ##Twist message
        self.move_cmd = Twist()

        #Trigger to makes a robot move
        self.move = False
        self.direction = None

        #Odometry list [pos_x, pos_y, orient_z]
        self.ref_odom = []
        self.cur_odom = []
        self.tar_odom = []
        self.goal_odom = [0,0,-185]

        #output from ICO
        self.ico_out = None

        #Parameters
        self.max_speed = 1.5
        self.reference = None
        self.lin_speed = None
        self.ang_speed = 1.5

        self.once = False

        #Node initialization
        self.init = None

        #Deceleration factor
        self.decel_factor = ''

        #Publishers
        #MOVO Physical Output pubs
        self.motion_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1, latch=False)


        rospy.Subscriber('/signal/init',Bool, self.init_cb, queue_size = 1)
        rospy.Subscriber('/robot/move', Bool, self.move_cb, queue_size = 1)
        rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odom_cb, queue_size = 1)

    #Reference position must be captured via an initialization signal.
    def capture_ref(self):
        if self.init == False:
            #print("Waiting for a reference signal")
            pass
        else:

            if self.once == False:
                self.ref_odom = self.cur_odom[:]
                self.once = True
                print('Reference Odometry is collected')
                #Once a reference is obtained, generates the target odom.
                print('Target Odometry is generated')
                self.target_odom(self.goal_odom)
            else:
                #print("Reference is already captured")
                pass

    #target goal given by user
    def target_odom(self, goal_list):
        
        if (goal_list[2] < 0):
            self.direction = "CW"
        else:
            self.direction = "CCW"

        pos_x = self.ref_odom[0]+goal_list[0]
        pos_y = self.ref_odom[1]+goal_list[1]
        orient_z = self.ref_odom[2]+goal_list[2]

        if orient_z < 0 or orient_z > 360:
            orient_z = orient_z%360
        else:
            orient_z

        self.tar_odom = [pos_x, pos_y, orient_z]

    #main
    def output_main(self):
        self.capture_ref()                      #trigger once
        self.target()

    def target(self):

        diff = self.angle_difference(self.cur_odom, self.tar_odom)
        """
        if diff >=0: 
            speed = (self.max_speed-(self.max_speed*0))*self.decel_rate(diff, self.max_speed)
        else:
            print("Done")
            speed = 0

        if self.direction == "CW":
            #self.drive(speed)
            print(speed)
        elif self.direction == "CCW":
            #self.drive(-speed)
            print(-speed)
            #self.drive(speed)
        """
    #Method to publish output
    def drive(self,drive):
        print("Angular:  ", drive)
        self.motion_pub.publish(self.twist_body(0, 0, drive))
   
    #Speed message constructor
    def twist_body(self, linear_x, linear_y, angular_z):
        #Axis X/Y, linear move
        self.move_cmd.linear.x = linear_x
        self.move_cmd.linear.y = linear_y
        #Axis Z, angular turn
        self.move_cmd.angular.z = angular_z
        return self.move_cmd

    #This method needs to be reconsidered since the calculation is complicated than usual.
    def angle_difference(self, cur_list, tar_list):
        diff_z = cur_list[2] - tar_list[2]

        if self.direction == "CW":
            if diff_z < 0:
                diff_z = diff_z%360
            else:
                diff_z = abs(diff_z)

        elif self.direction == "CCW":
            if diff_z < 0:
                diff_z = abs(diff_z)
            else:
                diff_z = 360-diff_z

        print('cur: ', cur_list[2])
        print('tar: ', tar_list[2])
        print('diff: ',diff_z)


        return diff_z

    #angle conversion
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        #Service robot doesn't have roll and pich rotation, Only yaw is active
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        #print('Q2E Z: ', Z)

        #Convert into a range of 0 to 360
        if Z < 0:
            return Z%360
        else:
            return Z



    #Use initialization signal to capture reference point
    def init_cb(self, signal):
        self.init = signal.data

    def shutdown_cb(self, signal):
        #Signal to shutdown this from input node.
        if signal.data == True:
            rospy.signal_shutdown("Shutdown signal is received, turn this node off")

    #Signal from input node to order the robot to move
    def move_cb(self, signal):
        self.move = signal.data

    ##Decide on which output main should we throw
    #Wheel Odometry callback
    def odom_cb(self, value):
        pos_x = value.pose.pose.position.x
        pos_y = value.pose.pose.position.y
        orient_z = self.quaternion_to_euler(value.pose.pose.orientation.x, value.pose.pose.orientation.y, value.pose.pose.orientation.z, value.pose.pose.orientation.w)
        self.cur_odom = [pos_x, pos_y, orient_z]
        #print(self.cur_odom)
        self.output_main()

    #slow down when it reaches a certain distance
    def decel_rate(self, diff, max_pos):
        min_pos = max_pos*self.decel_factor
        if diff < min_pos: 
            result = 1-((diff-min_pos)/(max_pos-min_pos))
        else:
            result = 1
            return result

    #Note
    ##Kinova movo max speed is 2 m/s, but clamped at 0.5 m/s
    ##Ref: https://newatlas.com/kinova-robotics-movo/59883/
    ## Quaternion: Range of the value: linear x -1 to 1 linear y -1 to 1 angular z -1 to 1
    ##Ref: https://answers.ros.org/question/9697/error-assi  def __init__(self):
