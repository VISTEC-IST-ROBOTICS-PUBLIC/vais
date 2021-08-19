#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Float32
from ico.icocore_src import IcoCoreClass


if __name__ == "__main__":
  print("Initialize ICO Core")
  rospy.init_node("ICO_core")

  try:
    ##object instantiation  
    ico = IcoCoreClass()
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("ICO core node terminated.")
