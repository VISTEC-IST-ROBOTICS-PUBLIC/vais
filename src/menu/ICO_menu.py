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
        #Parameters
        self.move = False

        #Publishers
        #Control signals
        self.ar_capture_pub = rospy.Publisher('/signal/ar_capture', Bool, queue_size = 1)           
        self.odom_capture_pub = rospy.Publisher('/signal/odom_capture', Bool, queue_size = 1)          
        self.shutdown_pub = rospy.Publisher('/signal/shutdown', Bool, queue_size = 1)   #Once this node is triggered, all related nodes are closed

        #robot start/stop signals
        self.move_pub = rospy.Publisher('/robot/move', Bool, queue_size = 1)            #Trigger signal to allows a robot to move

        #references
        self.vais_pub = rospy.Publisher('/data/vais_param', vais_param, queue_size=1)   #All parameters are bundled in custom message and sent.

    def default_value(self):
        msg = vais_param()
        msg.state = 'Angular'
        msg.e_object = 1
        msg.p_object = 4
        msg.r_object = 7
        msg.l_rate = 0.05
        msg.goal_x = 3
        msg.goal_y =0
        msg.goal_z = 90
        msg.decel_factor = 0.5
        self.vais_pub.publish(msg)

    def input_ver(self, sys_version, st_value):
        #First load to store the robot learning state
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

    def statement(self, value):
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

    def dynamic_parameters(self):
        print("Waiting for dynamic reconfigure connection")
        client = dynamic_reconfigure.client.Client("movo/movo_driver", timeout=20)
  
        rcv_input = self.input_ver(sys.version_info[0], 0)
        if (rcv_input == 1):
            #experiment
            print("Experiment dynamic parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":1.5, "accel_limit_mps2":1.5, "yaw_rate_limit_rps":2.35, "yaw_accel_limit_rps2": 3.14})
        else:
            #safety
            print("Safety parameters are loaded")
            client.update_configuration({"x_vel_limit_mps":0.5, "accel_limit_mps2":1.0, "yaw_rate_limit_rps":1.0, "yaw_accel_limit_rps2": 1.0})
            

    def input_selection(self):
        rcv_input = self.input_ver(sys.version_info[0], 1)
        if rcv_input == 1:
            print('Load a default parameters')
            #Automatically publish to vais_parameters
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

    def shutdown(self):
        #if this invokes, trigger shutdown signal to everyone.
        self.shutdown_pub.publish(True)

    ##launcher have to trigger here
    def menu_learn(self):

        #First, rest all topics to be False state
        self.ar_capture_pub.publish(False)
        self.odom_capture_pub.publish(False)
        self.move_pub.publish(False)

        #Second, Load dynamic parameters
        self.dynamic_parameters()

        #Third, Load vais parameters
        self.input_selection()
        
        #Last, Robot's option
        self.option()

    def option(self):
        #Press detection
        while(True):

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
            if key == 'i':
                print("Press enter to capture ar_tag")
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

            elif key == 'p':
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


