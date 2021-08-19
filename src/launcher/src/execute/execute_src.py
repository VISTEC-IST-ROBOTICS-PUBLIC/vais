#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32, String
from ico_weight.icoweight_src import IcoWeightClass

class ICOExecuteClass(object):
  def __init__(self):

    #Subscribers
    #Properties Subs
    self.object = rospy.Subscriber('/object/object_name', String, self.objname_cb, queue_size = 1)
    self.lin_sub = rospy.Subscriber('/data/linear_maxspeed', Float32, self.maxlinear_cb, queue_size = 1)
    self.ang_sub = rospy.Subscriber('/data/angular_maxspeed', Float32, self.maxangular_cb, queue_size = 1)

    self.lin_maxspeed = 0
    self.ang_maxspeed = 0

  def maxlinear_cb(self, data):
    self.lin_maxspeed = data.data

  def maxangular_cb(self, data):
    self.ang_maxspeed = data.data

  def objname_cb(self, name):
    #Trigger loader
    self.loadicoweight(name.data)

  def loadicoweight(self, data):
    while data == '':
      print("Waiting for object")

    #pw_instant = IcoWeightClass()
    #status, w_predict_lin = pw_instant.load(data, "Linear")
    #status, w_predict_ang = pw_instant.load(data, "Angular")


    if self.lin_maxspeed != 0 and self.ang_maxspeed !=0:
      result_lin = self.maxspeed - (self.maxspeed - w_predict_lin)
      result_ang = self.maxspeed - (self.maxspeed - w_predict_ang)

      #Just for a test
      #result_lin = 0.1
      #result_ang = 0.1

    else:
      print("This is a default value")
      result_lin = 0.5
      result_ang = 0.5

    try:
      #Get a current value of parameter
      vel_lin_param = rospy.get_param('/move_base/EBandPlannerROS/max_vel_lin')
      vel_th_param = rospy.get_param('/move_base/EBandPlannerROS/max_vel_th')
      print("Got EBandPlanner parameters")
      print("Current linear velocity parameter: ", vel_lin_param)
      print("Current angular velocity parameter: ", vel_th_param)

      #set parameter with a new output
      rospy.set_param('/move_base/EBandPlannerROS/max_vel_lin', result_lin)
      rospy.set_param('/move_base/EBandPlannerROS/max_vel_th', result_ang)
      print("New EBandPlanner parameters is set")

      except:
        print("Error with parameters")
      

#Python test area
if __name__ == '__main__':
  pass