#!/usr/bin/env python
 
import rospy
import sys
import dynamic_reconfigure.client

def input_ver( sys_version):
    #First load to store the robot learning state
    input_state_chk = False
    while not input_state_chk:
        try:
            statement = "Enter 1 for MOVO experiment parameters 2 for safety default parameters: \n"
            #Python v 2.7
            if sys_version < 3:
                rcv_input = int(raw_input(statement))
                return rcv_input
            #Python v 3.x+
            else:
                rcv_input = int(input(statement))      
                return rcv_input  

        except ValueError:
            print('Please enter only value. Try again: ')

if __name__ == "__main__":
    rospy.init_node("Reconfig_movo_driver")

    input = input_ver(sys.version_info[0])

    client = dynamic_reconfigure.client.Client("movo/movo_driver", timeout=20)
    if (input == 1):
        #experiment
        client.update_configuration({"x_vel_limit_mps":1.5, "accel_limit_mps2":1.5, "yaw_rate_limit_rps":2.35, "yaw_accel_limit_rps2": 3.14})
    else:
        #safety
        client.update_configuration({"x_vel_limit_mps":0.5, "accel_limit_mps2":1.0, "yaw_rate_limit_rps":1.0, "yaw_accel_limit_rps2": 1.0})

    print("Update dynamic parameters")