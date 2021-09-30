#!/usr/bin/env python

#MOVO modules
#from system_defines import *
#Note use rqt_reconfigure to modify maximum velocity/acceleration of MOVO to 1.0m/s and 0.5m/s2

from movo_msgs.msg import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64, Bool, String
from nav_msgs.msg import Odometry
import message_filters
import rospy
import sys
import math
import numpy as np

class BaseOpClass(object):
  def __init__(self):

    #Parameters
    ##construct a twist message
    self.move_cmd = Twist()
    self.conf_cmd = ConfigCmd()

    #Trigger
    self.move = False

    #Odometry list [pos_x, pos_y, orient_z]
    self.ref_odom = []
    self.cur_odom = []
    self.tar_odom = []
    self.ico_out = {}

    #Parameters
    self.max_speed = None
    self.reference = False
    self.lin_speed = None
    self.ang_speed = None

    #state
    self.state = None
    self.init = None

    #Target example
    self.tar_odom = [5, 5 ,0]

    #Publishers
    #MOVO Physical Output Pubs
    self.motion_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1, latch=False)
    self.init_pub = rospy.Publisher('/signal/init',Bool, queue_size = 1)

    #Signal subs
    rospy.Subscriber('/signal/init',Bool, self.init_cb, queue_size = 1)
    rospy.Subscriber('/signal/shutdown', Bool, self.shutdown_cb, queue_size = 1)
    rospy.Subscriber('/signal/reference', Bool, self.reference_cb, queue_size = 1)

    #Robot information receievers Subs
    rospy.Subscriber('/robot/state', String, self.state_cb, queue_size = 1)
    rospy.Subscriber('/robot/move', Bool, self.move_cb, queue_size = 1)
    rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odom_cb, queue_size = 1)

    #Linear/Angular maximum speed Subs
    rospy.Subscriber('/data/lin_max', Float32, self.lin_cb, queue_size = 1)
    rospy.Subscriber('/data/ang_max', Float32, self.ang_cb, queue_size = 1)

    #ICO output
    rospy.Subscriber('/ico/output', Float32, self.ico_cb, queue_size = 1)

  #Reference position must be captured at the start of the node (Toggle)
  def capture_ref(self, data):
    if self.reference == False:
      #print("Waiting for a reference signal")
      pass
    else:
      if self.once == False:
        self.ref_odom = self.cur_odom.copy()
        self.once = True
        print('Reference Odometry is collected')
      else:
        #print("Reference is already captured")
        pass

  #target goal given by user
  def target_odom(self, target_list):
    pos_x = self.ref_odom[0]+target_list[0]
    pos_y = self.ref_odom[1]+target_list[1]
    orient_z = self.ref_odom[2]+target_list[2]
    self.tar_odom = [pos_x, pos_y, orient_z]

  #current odom should be used in a wheel_odom callback
  def current_odom(self, data):
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    orient_z = self.quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    self.cur_odom = [pos_x, pos_y, orient_z]

  def output_main(self,data):
    self.current_odom(data)                     #current_odom always trigger
    self.capture_ref(data)                      #trigger once
    self.target_odom(self.tar_odom)             #list


    ####Tracking
    if self.state == "linear":
      diff = self.linear_euclidean(self.cur_odom, self.target_odom)
    elif self.state == "angular":
      diff = self.angle_difference(self.cur_odom, self.target_odom)
    else:
      print("Error please check input state")

    #######this thing has to be looked up
    #1. difference between distance and angle (might need to use a normalization factor?)
    #2. speed has to be reduce to minimum value that MOVO can be barely drived and cut off to 0
    #3. Incase of continuous route, we might need to reset a reference onwards
    #4. speed and position matters
    
    speed = self.max_speed-(self.max_speed*self.ico_out)
    result_speed = self.speed_modifier(speed,diff, 2)
    self.publish_output(result_speed)

  #Method to publish output
  def publish_output(self,drive):
    if self.state == 'Linear':
      print("Linear: ", drive)
      self.motion_pub.publish(self.twist_body(drive, 0, 0))
    elif self.state == 'Angular':
      print("Angular:  ", drive)
      self.motion_pub.publish(self.twist_body(0, 0, drive))
    else:
      print("MOVO Output error: Please check a state command")

  #Speed message constructor
  def twist_body(self, linear_x, linear_y, angular_z):

    #construct a message
    #Axis X/Y, move
    self.move_cmd.linear.x = linear_x
    self.move_cmd.linear.y = linear_y

    #Axis Z, turn
    self.move_cmd.angular.z = angular_z

    return self.move_cmd

  def angle_difference(self, cur_list, tar_list):
    diff_z = tar_list[2] - cur_list[2]
    return diff_z

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

  def normalization(self, input, min, max):		
    nm = (input - min)/(max - min)
    return float(nm)

  def quaterniontoeuler(self, x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    #Only yaw is active
    return Z

  def negative_angle_converter(self, angle):
    #check output first
    if angle < 0:
      angle = 360+ angle
      return angle
    pass

  def target_distance(self, ref_dist, tar_dist):
    result = ref_dist + tar_dist
    return result

  def target_angle(self, ref_angle,tar_angle):
    result = ref_angle+tar_angle
    return result

  def init_cb(self, signal):
    self.init = signal.data
    return self.init

  def shutdown_cb(self, signal):
    #Signal to shutdown this from input node.
    if signal.data == True:
      rospy.signal_shutdown("Shutdown signal is received, turn this node off")

  def reference_cb(self, signal):
    #Manually triggered to capture a reference point from the input node
    self.reference = signal.data

  def state_cb(self, state):
    self.state = state.data

  def move_cb(self, signal):
    #Signal from input node to order the robot to move
    self.move = signal.data

  def lin_cb(self, value):
    self.lin_speed = value.data

  def ang_cb(self,value):
    self.ang_speed = value.data

  def odom_cb(self, value):
    self.output_main(value)

  def ico_cb(self, value):
    self.ico_out = value

  #slower down from factor% range
  def speed_modifier(self, speed, diff, pos_factor):
    if diff < diff/pos_factor: 
      result = speed*(1-(diff/pos_factor))
    else:
      result = speed
    return result


##Note Kinova movo move speed is 2 mps //clamp at 0.5 m/s
#https://newatlas.com/kinova-robotics-movo/59883/

#NOTE
#Range of the value: linear x -1 to 1 linear y -1 to 1 angular z -1 to 1
#Ref https://answers.ros.org/question/9697/error-assigning-a-python-quaternion/