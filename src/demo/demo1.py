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

import time


class Demo1(object):
    def __init__(self):

        #Instantiation
        self.move_cmd = Twist()
        self.dmm = Data()
        self.output = BaseMotion()
        
        rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odom_cb, queue_size = 1)

    # Move by x meter(s)
    def forward(self, target):
        state = "Linear"
        #Retreive marker ID, ETL to weight (One-time ALVAR markers obtained)
        alvar_msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)        
        #return highest weight among multiple items
        weight_result = self.etl_msg(alvar_msg, state)

        #speed conversion
        speed = self.speed_conversion(weight_result, state)
    
        self.output.move_forward(target, speed)
        self.output.motion_stop()

        time.sleep(1)

    def backward(self, target):
        state = "Linear"
        #Retreive marker ID, ETL to weight (One-time ALVAR markers obtained)
        alvar_msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)        
        #return highest weight among multiple items
        weight_result = self.etl_msg(alvar_msg, state)

        #speed conversion
        speed = self.speed_conversion(weight_result, state)
    
        self.output.move_backward(target, speed)
        self.output.motion_stop()

        time.sleep(1)


    #Look through this later
    # + is a CCW rotaion, - is a CW rotation.
    def turn(self, target):
        state = "Angular"
        #Retreive marker ID, ETL to weight (One-time ALVAR markers obtained)
        alvar_msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)        
        #return highest weight among multiple items
        weight_result = self.etl_msg(alvar_msg, state)

        #speed conversion
        speed = self.speed_conversion(weight_result, state)

        if target>=0:
            self.output.rotate_clock(target, speed)
            self.output.motion_stop()
        else:
            self.output.rotate_anticlock(target,speed)
            self.output.motion_stop()
        time.sleep(1)

           
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



    #Load weight from result folder
    def load(self,state,id):
        result = self.dmm.load_result(state,id)
        return result

    #Extract-Transform-Load from Alvar msg to weight
    def etl_msg(self, msg, state):
        etl_weight = {}
        if msg.markers:
            #ETL from msg to dict for each detected markers
            for index, value in enumerate(msg.markers):
                #Avoid nan value and unnecessary alvar_tag
                if value.id < 1 or value.id > 10 or math.isnan(value.pose.pose.position.x) or math.isnan(value.pose.pose.position.y) or math.isnan(value.pose.pose.position.z):
                    pass
                else:
                    weight = self.load(state, value.id)
                    etl_weight[value.id] = weight
            
            print("Weight dict: ", etl_weight)

            #return max available weight 
            find_max = max(etl_weight.keys(), key=lambda k:etl_weight[k])                         #Use winner takes all (most sensitve values are the chosen result to drive the robot)
            max_value = etl_weight[find_max]
            return max_value
            
        else:
            #Case of no markers
            print ("[ERROR]: No alvar markers detected")
            pass

    def demo_waypoint(self):
        #set a MOVO to the experiment parameters;
        self.dynamic_parameters(1)

        #set a MOVO to the safety parameters again
        self.dynamic_parameters(0)
        pass

    def dynamic_parameters(self, input):                                                                   #Read dynamic parameters from the robots and rewritten with defined values.
        if (input == 1):
            client = dynamic_reconfigure.client.Client("movo/movo_driver", timeout=20)

            #experiment
            print("[INFO]: Experiment dynamic parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":1.5, "accel_limit_mps2":1.5, "yaw_rate_limit_rps":3.14, "yaw_accel_limit_rps2": 2.35})
        elif (input ==0):
            #safety
            print("[INFO]: Safety parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":0.5, "accel_limit_mps2":1.0, "yaw_rate_limit_rps":1.0, "yaw_accel_limit_rps2": 1.0})
        else:
            print("[ERROR]: Please check input")

    def log(self, timestamp, vel_x, vel_y, ang_z):
        #Read wheel odometry

        self.dmm.save_feedback('waypoint',timestamp, vel_x, vel_y, ang_z)
        #Store value as CSV
        pass


    #Wheel Odometry callback
    def odom_cb(self, value):
        timestamp = value.header.stamp
        #timestamp = datetime.datetime.fromtimestamp(timestamp.to_sec())
        print(timestamp)
        vel_x = value.twist.twist.linear.x
        vel_y = value.twist.twist.linear.y
        ang_z = value.twist.twist.angular.z
        print(timestamp, vel_x, vel_y, ang_z)
        self.log(timestamp, vel_x, vel_y, ang_z)



if __name__ == "__main__":
    rospy.init_node("demo1")
    Demo = Demo1()
    #Set param to experimental mode
    Demo.dynamic_parameters(1)

    #time.sleep(10)
    print("Start moving")
    #Demo.forward(1.7)
    Demo.turn(90)
    #Demo.forward(3.1)
    #print("Pause")
    #time.sleep(20)
    #print("Resume")
    Demo.turn(-180)
    ##time.sleep(1000)
    #Demo.forward(3.0)
    #Demo.turn(90)
    #Demo.backward(1.7)

    rospy.signal_shutdown(True)
    rospy.spin()



    #Test ID_1 from result
    #Demo.load("Angular",1)
    #Demo.load("Linear",1)

