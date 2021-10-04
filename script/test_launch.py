#!/usr/bin/env python
import rospy
from test.test1 import test1

if __name__ == '__main__':
   print('Initialize ICO test node')
   # initialize ICO execute node
   rospy.init_node('ICO_test')
   try:
     angle_test = test1()
     rospy.spin()

   except rospy.ROSInterruptException:
     rospy.loginfo("ICO test node terminated.")

