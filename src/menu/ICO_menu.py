#!/usr/bin/env python

import rospy
import sys
import dynamic_reconfigure.client
from std_msgs.msg import Bool, String, Float32, Int32
from vais.msg import vais_param
import termios
import tty
import select
import time

class Menu(object):
    def __init__(self):
        #Initial Parameters
        self.move = False

        #Publishers
        self.ar_capture_pub = rospy.Publisher('/signal/ar_capture', Bool, queue_size = 1)           #ALVAR feedback from robot's camera.
        self.odom_capture_pub = rospy.Publisher('/signal/odom_capture', Bool, queue_size = 1)       #Odometry feedback from robot.   
        self.shutdown_pub = rospy.Publisher('/signal/shutdown', Bool, queue_size = 1)               
        self.move_pub = rospy.Publisher('/robot/move', Bool, queue_size = 1)                        #Trigger signal to allows robot to move
        self.vais_pub = rospy.Publisher('/data/vais_param', vais_param, queue_size=1)               #All parameters used in learning.

    def default_value(self):                                                                        #Default values
        msg = vais_param()
        msg.state = 'Angular'
        msg.ar_id = 10
        msg.e_object = 1
        msg.p_object = 3
        msg.r_object = 7
        msg.l_rate = 0.01
        msg.goal_x = 4
        msg.goal_y =0
        msg.goal_z = 90
        msg.decel_factor = 0.1                                                                   #90% of goal reach
        self.vais_pub.publish(msg)

    def input_ver(self, sys_version, st_value):                                                     #Recieves input from user, check python version in a machine.
        input_state_chk = False
        while not input_state_chk:
            try:
                statement = self.statement(st_value)
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

    def statement(self, value):                                                                     #Input statements for each questions.
        if value == 0:
            input_state = """
        Dynamic parameters configuration choices:
        1: MOVO experiment parameters
        2: Safety default parameters
        """
        if value == 1:
            input_state = """
        Please define an input command:
        1: Load default
        2: Manual input
        """
        elif value ==2:
            input_state = """
        Please define an input command:
        1: Linear
        2: Angular
        """
        elif value == 3:
            input_state = "Please enter an exemption range: "
        elif value == 4:
            input_state = "Please enter a predictive range: "
        elif value == 5:
            input_state = "Please enter a reflexive range: "
        elif value == 6:
            input_state = "Please enter a linear goal distance (forward movement): "
        elif value == 7:
            input_state = "Please enter a linear goal distance (perpendicular movement): "
        elif value == 8:
            input_state = "Please enter a linear angular goal (degree): "
        elif value == 9:
            input_state = "Please enter a learning rate: "
        elif value == 10:
            input_state = "Please enter a learning rate: "

        return input_state

    def dynamic_parameters(self):                                                                   #Read dynamic parameters from the robots and rewritten with defined values.
        print("Waiting for dynamic reconfigure connection")
        client = dynamic_reconfigure.client.Client("movo/movo_driver", timeout=20)
  
        #Use this in case of testing between optimal/default value
        #rcv_input = self.input_ver(sys.version_info[0], 0)

        #Experimental mode
        rcv_input = 1

        if (rcv_input == 1):
            #experiment
            print("Experiment dynamic parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":1.5, "accel_limit_mps2":1.5, "yaw_rate_limit_rps":3.14, "yaw_accel_limit_rps2": 2.35})
        else:
            #safety
            print("Safety parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":0.5, "accel_limit_mps2":1.0, "yaw_rate_limit_rps":1.0, "yaw_accel_limit_rps2": 1.0})
            
    def input_selection(self):                                                                      
        rcv_input = self.input_ver(sys.version_info[0], 1)
        if rcv_input == 1:
            print('Load a default parameters')
            self.default_value()
            return rcv_input
        elif rcv_input == 2:
            print('Manual input')
            self.manual_input()
            return rcv_input
        else: 
            print('Invalid input. Try again:')

    def manual_input(self):
        msg = vais_param()
        rcv_input = self.input_ver(sys.version_info[0], 2)
        if rcv_input == 1:
            msg.state = 'Linear'
        elif rcv_input == 2:
            msg.state = 'Angular'
        print(
            '''These following input is set for a sensitivity area of the object detection (According to a camera)
            e_thr is a safety area where we define an acceptance range that object can deviate.
            p_thr is a preemptive area where the predictive signal is notice signal where the object pass beyond the exemption area.
            r_thr is a limit area to generate a reflexive signal that is exceeded our safety length.
            Noted that the deviation is occcured in 3D space.
            ''')
        msg.e_object = self.input_ver(sys.version_info[0], 3)
        msg.p_object = self.input_ver(sys.version_info[0], 4)
        msg.r_object = self.input_ver(sys.version_info[0], 5)
        print(
            '''These following input is a pre-defined goal in translational movement and rotational movement.
            ''')
        msg.goal_x = self.input_ver(sys.version_info[0], 6)
        msg.goal_y = self.input_ver(sys.version_info[0], 7)
        msg.goal_z = self.input_ver(sys.version_info[0], 8)
        print(
            '''This setting is used for a learning rate
            ''')
        msg.l_rate = self.input_ver(sys.version_info[0], 9)
        print(
            '''This setting is to modify speed when the robot pass the certain point
            ''')                
        msg.decel_factor = self.input_ver(sys.version_info[0], 12)
        self.vais_pub.publish(msg)

    def shutdown(self):                                                                             #Trigger signal to shutdown all related nodes.
        self.shutdown_pub.publish(True)

    def menu_learn(self):                                                                           #Main structure for learning mechanisms which receives the user's input to manually captures the initial position of AR tag and odometry.

        #Initially resetted all topics to False state.
        self.ar_capture_pub.publish(False)
        self.odom_capture_pub.publish(False)
        self.move_pub.publish(False)

        #Load dynamic parameters from robot.
        self.dynamic_parameters()

        #Use this to test on multiple parameters tuning
        #Load VAIS parameters or entering it manually.
        #self.input_selection()
        

        #Skip manual input
        self.default_value()

        #Choose the option to operate the robot.
        self.option()

    def option(self):
        while(True):                                                                                #Indefinite loop

            #Main message
            msg = '''
                Option: 
                1. Press I to Initiate
                2. Press P to Pause
                3. Press R to Resume
                4. Press S to Shutdown
                5. Press ESC to exit
            '''
            print(msg)

            key = self.getKey()
            if key == 'i' or key == 'I':
                print("Press enter to capture AR tag")
                self.press_enter()
                self.ar_capture_pub.publish(True)

                print("Press enter to capture Robot's Odometry")
                self.press_enter()
                self.odom_capture_pub.publish(True)

                print("Press enter to trigger a robot start signal")
                self.press_enter()
                self.move_pub.publish(True)

                print("Press enter to trigger a robot stop signal")
                self.press_enter()
                self.move_pub.publish(False)

                print("One second Cooldown")
                rospy.sleep(1)

                print('reset capture to False state')
                self.ar_capture_pub.publish(False)
                self.odom_capture_pub.publish(False)

                print("Press enter to back to a menu section")
                self.press_enter()

            elif key == 'p'or key == 'P':
                print("Pause all related nodes") 
                self.ar_capture_pub.publish(False)
                self.odom_capture_pub.publish(False)
                self.move_pub.publish(False)
                print("Press enter to back to a menu section")
                self.press_enter()

            elif key == 'r':
                print("Resume all related nodes") 
                self.ar_capture_pub.publish(False)
                self.odom_capture_pub.publish(False)
                self.move_pub.publish(True)
                print("Press enter to back to a menu section")
                self.press_enter()                

            elif key == 's':
                print("Shutdown all related nodes") 
                self.ar_capture_pub.publish(False)
                self.odom_capture_pub.publish(False)
                self.move_pub.publish(False)
                self.shutdown()
                rospy.signal_shutdown("Shutdown all nodes") 
                break

            elif key =='\x1b':                      #ESC
                self.ar_capture_pub.publish(False)
                self.odom_capture_pub.publish(False)
                self.move_pub.publish(False)
                self.shutdown()
                rospy.signal_shutdown("Exit")
                sys.exit()

    def press_enter(self):
        #Python v 2.7
        if sys.version_info[0] < 3:
            text = raw_input("")
        #Python v 3.x+
        else:
            text = input("")      

        if text == "":
            pass

    def getKey(self):
        #Launch termios setting
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


