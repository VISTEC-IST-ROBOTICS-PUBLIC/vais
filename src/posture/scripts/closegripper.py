#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from movo_action_clients.head_action_client import HeadActionClient
from movo_action_clients.jaco_action_client import JacoActionClient
from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from std_msgs.msg import Bool
import datetime as dt

if __name__ == "__main__":

    print("initialize gripper to close")
    process_start_time = dt.datetime.now()
    rospy.init_node("ico_opengripper")
    dof = rospy.get_param('~jaco_dof')
    sim = rospy.get_param("~sim", False)
    if (sim):
        rospy.wait_for_message('/sim_initialized',Bool)


    movo_lfinger = GripperActionClient('left')
    movo_rfinger = GripperActionClient('right')
    rospy.sleep(2)

#--------------------------------finger----------------------------------------------------
    movo_lfinger.command(0)#open
    movo_rfinger.command(0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)



