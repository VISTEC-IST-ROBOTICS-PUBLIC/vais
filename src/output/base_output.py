#!/usr/bin/env python

#MOVO modules
#from system_defines import *
from movo_msgs.msg import *
from std_msgs.msg import Bool
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

    #Output from a previous node
    self.out = None
    self.move = False

    #Reference Odometry
    self.ref_euler = {}

    #Current Odometry
    self.curodom_x = None
    self.curodom_y = None
    self.curodom_z = None
    self.curodom_w = None
    self.cureuler_z = None

    #Parameters
    self.max_speed = 1.5         #maximum speed       
    self.reference = False

    self.lin_speed = None
    self.ang_speed = None

    self.state = None
    
    #Counter flag
    self.ct = 0
    self.limit_ct = 0
    self.lower_ct = 0
    self.upper_ct = 0

    self.init = None

    #Distance move
    self.dist = None

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
    rospy.Subscriber('/movo/feedback/wheel_odometry', Odometry, self.odomcb, queue_size = 1)

    #Linear/Angular maximum speed Subs
    rospy.Subscriber('/data/lin_max', Float32, self.lin_cb, queue_size = 1)
    rospy.Subscriber('/data/ang_max', Float32, self.ang_cb, queue_size = 1)

  #Note use rqt_reconfigure to modify maximum velocity/acceleration to 1.0m/s and 0.5m/s2


##ref odom, cur odom, target_odom

  def capture_refodom(self, data):
    #Reference position must be captured at the start of the node
    if self.reference == False:
      #print("Waiting for a reference signal")
      pass
    else:
      if self.once == False:
        self.refodom[0] = data.pose.pose.position.x
        self.refodom[1] = data.pose.pose.position.y
        self.refodom[2] = data.pose.pose.orientation.z
        self.refodom[3] = data.pose.pose.orientation.w
        self.ref_euler = self.quaterniontoeuler(self.refodom_x, self.refodom_y, self.refodom_z, self.refodom_w)
        self.once = True
        print('Reference Odometry is collected')
      else:
        #print("Reference is already captured")
        pass


    
    if self.init == False:
      pass
    else:
      if self.move == False:
        pass
      else:
        self.move_base(float(value.output))

  def move_base(self, value):

      #Output parameter must be matched with MOVO speed on rqt_reconfigure
      if self.state == 'Linear':
        self.max_speed = self.lin_speed
        #current distance from goal
        result_lin = self.linear_euclidean(self.refodom_x, self.refodom_y, self.curodom_x, self.curodom_y)

        speed = self.speed_modifier()

        self.dist = result_lin
        if result_lin > 5:
          print("Warning: Reach linear distance limit (4 meter)")
          if self.limit_ct < 2:
            self.limit_ct += 1
          else:
            self.init_pub.publish(False)
            self.move = False

      elif self.state == 'Angular':
        self.max_speed = self.ang_speed
        result_ang = self.angle_difference(self.ref_euler,self.cureuler_z)
        print(self.cureuler_z)
        self.dist = self.cureuler_z

      else:
        print("Error: Please check maximum speed parameter")
        self.dist = 0

      drive = self.max_speed - (self.max_speed*value)                


      #Send info to log
      if drive is not None and self.dist is not None:
        self.output_pub(drive, self.dist)

      #Debugging purpose
      print('Drive value: ', drive)

      #Warning when Drive output is lower than 0
      if drive < 0:
        drive = 0
        print("Warning: Drive output is lower than 0")
        if self.lower_ct < 5:
          self.lower_ct += 1
        else:
          self.init_pub.publish(False)
          self.move = False

      #Speed cap than might be generated from ICO
      elif drive > self.max_speed:
        drive = self.max_speed
        print("Warning: Drive output is greater than max speed")
        
        if self.upper_ct < 5:
          self.upper_ct += 1
        else: 
          self.init_pub.publish(False)
          self.move = False

      else:

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

  def angle_difference(self, ref_z, cur_z):
    diff_z = ref_z-cur_z
    square_z = np.power(diff_z, 2)
    euc_result = math.sqrt(square_z)

  def linear_euclidean(self, ref_x, ref_y, cur_x, cur_y):
    #First, find the different
    diff_x = ref_x-cur_x
    diff_y = ref_y-cur_y

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

  def odomcb(self, value):

    #This callback keeps capturing the current odom of the robot
    self.curodom_x = value.pose.pose.position.x
    self.curodom_y = value.pose.pose.position.y
    self.curodom_z = value.pose.pose.orientation.z
    self.curodom_w = value.pose.pose.orientation.w
    self.cureuler_z = self.quaterniontoeuler(self.curodom_x, self.curodom_y, self.curodom_z, self.curodom_w)

    #Trigger reference odometry
    self.capture_refodom(value)

  #slower down from factor% range
  def speed_modifier(self, speed,cur_pos, max_pos, pos_factor):
    if cur_pos < max_pos/pos_factor: 
      result = speed*(1-(cur_pos/(max_pos/pos_factor)))
    else:
      result = speed
    return result

  #need?
  def outcb(self, value):

##Note Kinova movo move speed is 2 mps //clamp at 0.5 m/s
#https://newatlas.com/kinova-robotics-movo/59883/

#NOTE
#Range of the value: linear x -1 to 1 linear y -1 to 1 angular z -1 to 1
#Ref https://answers.ros.org/question/9697/error-assigning-a-python-quaternion/



