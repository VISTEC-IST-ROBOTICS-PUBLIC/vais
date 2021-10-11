#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from movo_action_clients.head_action_client import HeadActionClient
#from movo_action_clients.jaco_action_client import JacoActionClient
#from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from std_msgs.msg import Bool
import datetime as dt

if __name__ == "__main__":

    print("[INFO]: Initialize standby position")
    process_start_time = dt.datetime.now()
    rospy.init_node("ICO_standby")
   
    movo_head = HeadActionClient()
    movo_torsor = TorsoActionClient()
    rospy.sleep(2)

    print("[INFO]: Start adjustment")
    movo_torsor.clear()
    temp_torso = rospy.wait_for_message("/movo/linear_actuator/joint_states", JointState)
    current_torso_pos = list(temp_torso.position)
    movo_torsor.add_point(current_torso_pos, 0.0)
    # movo_torsor.add_point([0.45], 4)
    #movo_torsor.add_point([0.26], 6)     #This is updated as a base installation
    #movo_torsor.add_point([0.47], 12)    #Tallest possible
    movo_torsor.start()
    movo_torsor.wait(10.0)
    print("[INFO]: Body adjustment is done")

    tmp_head = rospy.wait_for_message("/movo/head/joint_states", JointState)
    current_angles= tmp_head.position

    movo_head.clear()
    head_timer = 0.0
    movo_head.add_point(list(current_angles), 0.0)
    head_timer += 10
    #movo_head.add_point([0.0, math.radians(-45.0)], head_timer)  #This is second phase
    movo_head.add_point([0.07392589002847672, -1.1551105976104736], head_timer) #This is a third phase
    movo_head.start()
    movo_head.wait(head_timer+5)
    print("[INFO]: Head adjustment is done")
