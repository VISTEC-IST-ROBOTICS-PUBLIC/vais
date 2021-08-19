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

    print("initialize standby")
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
        
        larm_tuck_deg = [92, 86, -23, 155, 0, -29, 97]
        rarm_tuck_deg = [-1.0 * x for x in larm_tuck_deg]

        larm_tray = [0.05169515255174595, 1.4020003832157124, -0.1250157057871526, 1.730909156160286, -0.1067778460100719, -1.266139752837538, 2.9084052032780185]
        rarm_tray = [-1.0 * x for x in larm_tray]
   
        #head = [0.03958948701620102, -0.898326575756073]
        head = [-0.19449278712272644, -0.8370144963264465]


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
    movo_larm.add_point(larm_tray, from_start_time)
    movo_rarm.add_point(rarm_tray, from_start_time)

    movo_larm.start()
    movo_rarm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)

#--------------------------------finger----------------------------------------------------
    movo_lfinger.command(0.165)#open
    movo_rfinger.command(0.165)
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

#-------------------------------finger------------------------------------------------------

#    while(True):
#      dummy_input = input('Press enter to continue:' )
#      if dummy_input == NULL:
#        break

    print('check before closing the finger')
#trigger something
    movo_lfinger.command(0.0)#close
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)


