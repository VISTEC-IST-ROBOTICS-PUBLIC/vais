#!/usr/bin/env python
import rospy
from menu.ICO_menu import Menu


if __name__ == '__main__':
   print('Initialize ICO menu node')
   #Initialize ICO learning node
   rospy.init_node('ICO_menu')
   try:
        ICO_menu=Menu()
        ICO_menu.menu_learn()
        #One time
        #rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("ICO menu node terminated.")

