#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from movo_action_clients.gripper_action_client import GripperActionClient
from std_msgs.msg import Bool
import datetime as dt

if __name__ == "__main__":

    print("initialize gripper to open")
    process_start_time = dt.datetime.now()
    rospy.init_node("movo_open_gripper")

    #movo_lfinger = GripperActionClient('left')
    movo_rfinger = GripperActionClient('right')
    rospy.sleep(2)

#--------------------------------finger----------------------------------------------------
    #movo_lfinger.command(0.165)#open
    movo_rfinger.command(0.165)
    #movo_lfinger.wait(3)
    movo_rfinger.wait(3)



