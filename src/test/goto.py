#!/usr/bin/env python

import rospy

#movebase action cleint is lost
from output.base_motion import BaseMotion


if __name__ == "__main__":
    rospy.loginfo("MOVE TEST")
    # rospy.init_node("move_closer", log_level=rospy.DEBUG)
    rospy.init_node("move_TEST", log_level=rospy.INFO)
    b_test = BaseMotion()
    print ("start base test")
    
    #b_test.rotate_anticlock(180,2.34)
    #b_test.motion_stop()
    #b_test.rotate_clock(180,1.25)
    #b_test.motion_stop()
    rospy.signal_shutdown(True)

    rospy.spin()