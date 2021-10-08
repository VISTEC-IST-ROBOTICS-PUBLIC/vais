#!/usr/bin/env python

import rospy
from demo.demo1 import Demo1


if __name__ == '__main__':
   print('Initialize ICO demo test node')
   # initialize ICO demo node
   rospy.init_node('ICO_demo')
   try:
     exe = Demo1
     rospy.spin()

   except rospy.ROSInterruptException:
     rospy.loginfo("ICO demo node terminated.")

