#!/usr/bin/env python

#MOVO modules
#from system_defines import *

from movo_msgs.msg import *
from geometry_msgs.msg import Twist
from vais.msg import vais_param
from std_msgs.msg import Float32, Float64, Bool, String
from nav_msgs.msg import Odometry
import rospy
import math
import numpy as np

class MOVO_output(object):
    def __init__(self):

        #Instantiation
        self.move_cmd = Twist()

        #Initial parameters
        self.move = False
        self.direction = None
        self.ico_out = None
        self.max_speed = None
        self.reference = None
        self.lin_speed = None
        self.ang_speed = None
        self.state = None
        self.odom_capture = False
        self.decel_factor = None

        #Odometry list [pos_x, pos_y, orient_z]
        self.ref_odom = []
        self.cur_odom = []
        self.tar_odom = []
        self.goal_odom = []

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
    def ref_capture(self, goal_odom):
        if self.odom_capture == True:

            self.ref_odom = self.cur_odom[:]
            print("[INFO]: Reference Odometry is collected at: ", self.ref_odom)
            #Once a reference is obtained, generates the target odom.
            self.target_odom(goal_odom)
            print("[INFO]: Target Odometry is generated at: ", self. tar_odom)
            self.odom_capture=False
            #Signal back to make ref_odom unrewritable
            self.odom_capture_pub.publish(self.odom_capture)

        else:
            #print("[INFO]: Waiting for a reference signal")
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
    def output_main(self, state, ico_out, goal_odom):
        self.ref_capture(goal_odom)                      #trigger once
        self.target(state, ico_out)
       
    def target(self, state, ico_out):
    
        #Tracking
        if state == "Linear":
            diff = self.linear_euclidean(self.cur_odom, self.tar_odom)
            max_diff = self.linear_euclidean(self.ref_odom, self.tar_odom)
        elif state == "Angular":
            diff = self.angle_difference(self.cur_odom, self.tar_odom)
            max_diff = self.angle_difference(self.ref_odom, self.tar_odom)
        else:
            diff = 0
            print("Error please check input state")

        print('cur: ', self.cur_odom)
        print('tar: ', self.tar_odom)

        #when diff > 2% of max difference (Threshold)
        threshold = 0.02*max_diff
        print('diff: ', diff)
        print('thres: ', threshold)
        if diff >= threshold:
                max = self.max_speed
                ico = float(self.ico_out)
                rate = self.decel_rate(diff, max_diff)
                speed = (max-(max*ico))*rate
                #min speed cap, else MOVO moves too slow to approach, around 5% of the robot's max speed
                min_cap = 0.025*max
                if speed < min_cap:
                    speed = min_cap

        else:
            print("Done")
            speed = 0

        print('speed: ', speed)
        

        if self.direction == "CCW":
            self.drive(speed)
            print(speed)
        elif self.direction == "CW":
            self.drive(-speed)
            print(-speed)

    #Method to publish output
    def drive(self,drive):
        #Safety
        if self.move == False:
            pass
        else: 
            if self.state == 'Linear':
                print("Linear: ", drive)
                self.motion_pub.publish(self.twist_body(drive, 0, 0))
            elif self.state == 'Angular':
                print("Angular:  ", drive)
                self.motion_pub.publish(self.twist_body(0, 0, drive))
            else:
                #print("MOVO Output error: Please check a state command")
                pass

    #Speed message constructor
    def twist_body(self, linear_x, linear_y, angular_z):
        #Axis X/Y, linear move
        self.move_cmd.linear.x = linear_x
        self.move_cmd.linear.y = linear_y
        #Axis Z, angular turn
        self.move_cmd.angular.z = angular_z
        return self.move_cmd

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

        #print('cur: ', cur_list[2])
        #print('tar: ', tar_list[2])
        #print('diff: ',diff_z)
        
        return diff_z

    #Wrongly calculated
    def linear_euclidean(self, cur_list, tar_list):
        #First, find the different
        diff_x = tar_list[0] - cur_list[0]
        diff_y = tar_list[1] - cur_list[1]

        #Second, square them
        square_x = np.power(diff_x, 2)
        square_y = np.power(diff_y, 2)

        #Last, sum and square root
        euc_result = math.sqrt(square_x+square_y)
        return euc_result

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

        #print('Q2E Z: ', Z)

        #Convert into a range of 0 to 360
        if Z < 0:
            return Z%360
        else:
            return Z
    
    #Use initialization signal to capture reference point
    def odom_capture_cb(self, signal):
        self.odom_capture = signal.data

    def shutdown_cb(self, signal):
        #Signal to shutdown this from input node.
        if signal.data == True:
            rospy.signal_shutdown("Shutdown signal is received, turn this node off")

    #Signal from input node to order the robot to move
    def move_cb(self, signal):
        self.move = signal.data

    #Wheel Odometry callback
    def odom_cb(self, value):
        pos_x = value.pose.pose.position.x
        pos_y = value.pose.pose.position.y
        orient_z = self.quaternion_to_euler(value.pose.pose.orientation.x, value.pose.pose.orientation.y, value.pose.pose.orientation.z, value.pose.pose.orientation.w)
        self.cur_odom = [pos_x, pos_y, orient_z]

        #Also trigger main output
        if self.state != None and self.ico_out != None and self.goal_odom!=None:
            self.output_main(self.state, self.ico_out, self.goal_odom)

    #Output from learning
    def ico_cb(self, value):
        self.ico_out = value.data

    #VAIS parameters
    def vais_cb(self, value):
        self.goal_odom = [value.goal_x, value.goal_y, value.goal_z]
        self.decel_factor = value.decel_factor
        self.state = value.state
        if value.state == 'Linear':
            self.max_speed = self.lin_speed
        elif value.state == 'Angular':
            self.max_speed = self.ang_speed
        else:
            #print('Error: Please check robot state')
            pass

    #Active configuration
    def aconf_cb(self, value):
        self.lin_speed = value.x_vel_limit_mps
        self.ang_speed = value.yaw_rate_limit_rps
        #print(self.lin_speed)
        #print(self.ang_speed)

    #slow down when it reaches a certain distance
    def decel_rate(self, diff, max_diff):
        min_diff = max_diff*self.decel_factor
        if diff < min_diff: 
            num = abs(diff-min_diff)
            denom = max_diff-min_diff
            result = 1.0-(num/denom)
        else:
            result = 1.0
        
        #print('Decel rate: ', result)
        return result


    #Note
    ##Kinova movo max speed is 2 m/s, but clamped at 0.5 m/s
    ##Ref: https://newatlas.com/kinova-robotics-movo/59883/
    ## Quaternion: Range of the value: linear x -1 to 1 linear y -1 to 1 angular z -1 to 1
    ##Ref: https://answers.ros.org/question/9697/error-assi  def __init__(self):
