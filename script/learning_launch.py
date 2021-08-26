#!/usr/bin/env python
import rospy
#import sys
#sys.path.append("..")

from ICO.main import Core as core


if __name__ == '__main__':
   print('Initialize ICO learning node')
   #Initialize ICO learning node
   rospy.init_node('ICO_learn')
   try:
        ICO_learn=core()
        rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("ICO Learning node terminated.")




