#!/usr/bin/env python

import rospy
from output.base_output import MOVO_output


if __name__ == '__main__':
   print('Initialize MOVO base output node')
   #Initialize ICO learning node
   rospy.init_node('MOVO_base_output')
   try:
        MOVO_out=MOVO_output()
        rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("ICO MOVO base output node terminated.")

