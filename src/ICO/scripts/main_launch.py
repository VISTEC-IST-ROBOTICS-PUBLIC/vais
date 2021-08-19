#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from signal_generator.icosignal_src import IcoSignalClass

if __name__ == "__main__":
  print("Initialize ICO signal")
  rospy.init_node("ICO_signal_generator")

  try:
    icosig = IcoSignalClass()
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("ICO signal_generator node terminated.")
