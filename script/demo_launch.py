#!/usr/bin/env python
import rospy
from demo.demo3 import demo3


if __name__ == '__main__':
   print('Initialize ICO demo test node')
   # initialize ICO demo node
   rospy.init_node('ICO_demo')
   try:
     exe = demo3()
     rospy.spin()

   except rospy.ROSInterruptException:
     rospy.loginfo("ICO demo node terminated.")

