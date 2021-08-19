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

    print("initialize tuckin")
    process_start_time = dt.datetime.now()
    rospy.init_node("ico_standby")
    dof = rospy.get_param('~jaco_dof')
    sim = rospy.get_param("~sim", False)
    if (sim):
        rospy.wait_for_message('/sim_initialized',Bool)

    movo_head = HeadActionClient()
    movo_larm = JacoActionClient(arm='left', dof=dof)
    movo_rarm = JacoActionClient(arm='right', dof=dof)
    movo_lfinger = GripperActionClient('left')
    movo_rfinger = GripperActionClient('right')
    movo_torsor = TorsoActionClient()
    rospy.sleep(2)

#-----------------------------------------arm-----------------------------------------------
    if "7dof" == dof:
        
        larm_tuck = [1.5941320095306066, 1.3632146377320162, -0.42622987771947507, 2.6089604355739455, 0.011976768054303477, -0.4517056767071738, 1.661984826731973]
        rarm_tuck = [-1.0 * x for x in larm_tuck]
  
        head = [0.029692914336919785, 0.02460547722876072]

    movo_larm.clear()
    movo_rarm.clear()
    tmp_left = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    current_larm_pos = list(tmp_left.position)
    tmp_right = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_rarm_pos = list(tmp_right.position)

    from_start_time = 0.0
    movo_larm.add_point(current_larm_pos, from_start_time)
    movo_rarm.add_point(current_rarm_pos, from_start_time)

    from_start_time += 4.0
    movo_larm.add_point(larm_tuck, from_start_time)
    movo_rarm.add_point(rarm_tuck, from_start_time)

    movo_larm.start()
    movo_rarm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)

#--------------------------------finger----------------------------------------------------
    movo_lfinger.command(0.0)
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

#----------------------------------------Head-----------------------------------------------
    tmp_head = rospy.wait_for_message("/movo/head/joint_states", JointState)
    current_angles= tmp_head.position

    movo_head.clear()
    time_from_start = 0.0
    movo_head.add_point(list(current_angles), 0.0)
    time_from_start += 2
    movo_head.add_point(head, time_from_start)
    movo_head.start()
    movo_head.wait(time_from_start+2)
