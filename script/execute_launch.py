#!/usr/bin/env python
import rospy
from execute.execute_src import ICOExecuteClass


if __name__ == '__main__':
   print('Initialize ICO execute node')
   # initialize ICO execute node
   rospy.init_node('ICO_Execute')
   try:
     exe = ICOExecuteClass()
     rospy.spin()

   except rospy.ROSInterruptException:
     rospy.loginfo("ICO Execute node terminated.")

