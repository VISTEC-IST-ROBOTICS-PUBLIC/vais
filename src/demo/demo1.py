#!/usr/bin/env python

import rospy
import math
import dynamic_reconfigure.client
from output.base_output import MOVO_output
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from ICO.data_management import Data
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time


class Demo1(object):
    def __init__(self):

        #Instantiation
        self.move_cmd = Twist()
        self.weight_load = Data()
        self.output = MOVO_output()

        #Publishers
        self.odom_capture_pub = rospy.Publisher('/signal/odom_capture',Bool, queue_size = 1)
        self.move_pub = rospy.Publisher('/robot/move',Bool, queue_size = 1)
        self.motion_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1, latch=False)
 
        #subscriber(s)
        rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.output.odom_cb, queue_size = 1)

    # Move by x meter(s)
    def forward(self, target):
        state = 'Linear'
        #Retreive marker ID, ETL to weight (One-time ALVAR markers obtained)
        alvar_msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)        
        #return highest weight among multiple items
        weight_result = self.etl_msg(alvar_msg, state)

        print(weight_result)

        #alternative method to receive message
        #self.output.exec_op(state, target, weight_result)
       

        #Write more on output module
        #self.output.motion_pub.publish(self.output.twist_body(speed_result, 0,0))


    #Look through this later
    # + is a CCW rotaion, - is a CW rotation.
    def turn(self, target):
        state = 'Angular'
        speed_result = 0
        #check this?
        self.output.motion_pub.publish(self.output.twist_body(0,0, speed_result))
           
           
    #Load weight from result folder
    def load(self,state,id):
        result = self.weight_load.load_result(state,id)
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

if __name__ == "__main__":
    rospy.init_node("demo1")
    Demo = Demo1()
    #Test ID_1 from result
    #Demo.load("Angular",1)
    #Demo.load("Linear",1)
    print("Dummy2")
    Demo.forward(1)
    rospy.spin()

#Every trigger    
#See once
#Load weight (if multi weight, get once)

#Loop check with Odom
    #Finish each task
#Finish

#active config
#set to MOVO drive