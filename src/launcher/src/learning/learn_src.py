#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool, String, Float32, Int32
import termios
import tty
import select

class ICOLearningClass(object):
  def __init__(self):
    #Publishers

    #Control signals
    self.init_pub = rospy.Publisher('/signal/init', Bool, queue_size = 1)           #if init is True, learning continues, else learning stops
    self.shutdown_pub = rospy.Publisher('/signal/shutdown', Bool, queue_size = 1)   #Once this node is triggered, all related nodes are closed
    self.reset = rospy.Publisher('/signal/reset', Bool, queue_size = 1)             #Need a further investigation to reset/respawn a node

    #robot start/stop signals
    self.move_pub = rospy.Publisher('/robot/move', Bool, queue_size = 1)            #Trigger signal to allows a robot to move

    #references
    self.state_pub = rospy.Publisher('/robot/state', String, queue_size = 1)
    self.l_rate_pub = rospy.Publisher('/data/l_rate', Float32, queue_size = 1)

    #BaseOp reference
    self.linmax_pub = rospy.Publisher('/data/lin_max', Float32, queue_size = 1)
    self.angmax_pub = rospy.Publisher('/data/ang_max', Float32, queue_size = 1)

    #ARUCO references
    self.markerid_pub = rospy.Publisher('/data/marker_id', Int32, queue_size = 1)
    self.markersize_pub = rospy.Publisher('/data/marker_size', Int32, queue_size = 1)

    #parameters for ICO signal generator
    self.reference_pub = rospy.Publisher('/signal/reference', Bool, queue_size = 1)  
    self.p_thr_pub = rospy.Publisher('/data/p_thr', Float32, queue_size = 1)
    self.r_thr_pub = rospy.Publisher('/data/r_thr', Float32, queue_size = 1)

    #move parameter
    self.move = False

  def default_load(self):
      input_state = """
      Please define an input command:
      1: Load default
      2: Manual input
      """
      input_state_chk = False
      while not input_state_chk:
        try:
          if sys.version_info[0] < 3:
            rcv_input = int(raw_input(input_state))
          else:
            rcv_input = int(input(input_state))

          if rcv_input == 1:
            print('Load default paramters')
            return rcv_input
          elif rcv_input == 2:
            print('Manual input')
            return rcv_input
          else: 
            print('Invalid input. Try again:')

        except ValueError:
          print('Please enter only value. Try again: ')

  #Note this aruco marker is implemented because of the flaw in an aruco detection.
  #This marker stores only one object at a time
  def aruco_marker(self):
    input_obj_chk = False
    while not input_obj_chk:
      try:
        #Note for Python2. When you input nothing and press Enter. It is expected to have Syntax Error.
        if sys.version_info[0] < 3:
          id_input = int(raw_input("Please enter a marker ID"))
          size_input = int(raw_input("Please enter a marker size"))
        else:   
          id_input = int(input("Please enter a marker ID"))
          size_input = int(input("Please enter a marker size"))
        
        return id_input,size_input

      except ValueError:
        print('Please enter only value. Try again: ')

  def max_speed(self):
    input_obj_chk = False
    while not input_obj_chk:
      try:
        #Note for Python2. When you input nothing and press Enter. It is expected to have Syntax Error.
        if sys.version_info[0] < 3:
          lin_input = int(raw_input("Please enter a linear maximum speed"))
          ang_input = int(raw_input("Please enter an angular maximum speed"))
        else:
          lin_input = int(input("Please enter a linear maximum speed"))
          ang_input = int(input("Please enter an angular maximum speed"))
        
        return lin_input, ang_input
      
      except ValueError:
        print('Please enter only value. Try again: ')

  def state_cmd(self):
    input_state = """
    Please define an input command:
    1: Linear
    2: Angular
    """
    input_state_chk = False
    while not input_state_chk:
      try:
        if sys.version_info[0] < 3:
          rcv_input = int(raw_input(input_state))
        else:
          rcv_input = int(input(input_state))

        if rcv_input == 1:
          print('State_cmd: Linear')
          return 'Linear'
        elif rcv_input == 2:
          print('State_cmd: Angular')
          return 'Angular'
        else:
          print('Invalid input. Try again:')

      except ValueError:
        print('Please enter only value. Try again: ')

  def l_rate(self):

    input_mode_chk = False
    while not input_mode_chk:
      print(
        '''This setting is used for a learning rate
        ''')
      try:
        if sys.version_info[0] < 3:
          lrate_input = float(raw_input("Please put a learning rate: "))
          print("Learning rate is: ", lrate_input)
          return lrate_input
        else:
          lrate_input = float(raw_input("Please put a learning rate: "))
          print("Learning rate is: ", lrate_input)
          return lrate_input

      except ValueError:
        print('Please enter only value. Try again: ')

  def obj_sensitivity(self):

    input_mode_chk = False
    while not input_mode_chk:
      print(
        '''This input is set for a sensitivity of the object detection (According to a camera)
        p_thr is a safety area where we define an acceptance range that object can deviate.
        r_thr is a limit area to generate a reflexive signal that is exceeded our safety length
        ''')
      try:
        if sys.version_info[0] < 3:
          pthr_input = int(raw_input("Please put a safety area threshold: "))
          print("Safety area Threshold is: ", pthr_input)
          rthr_input = int(raw_input("Please put a reflexive threshold: "))
          print("Reflexive area Threshold is: ", rthr_input)
          return pthr_input, rthr_input
        else:
          pthr_input = int(input("Please put a safe area threshold"))
          print("Safety area Threshold is: ", pthr_input)
          rthr_input = int(input("Please put a reflexive threshold"))
          print("Reflexive area Threshold is: ", rthr_input)
          return pthr_input, rthr_input

      except ValueError:
        print('Please enter only value. Try again: ')

  def init(self):
    self.init_pub.publish(True)

  def shutdown(self):
    #if this invokes, trigger shutdown signal to everyone.
    self.shutdown_pub.publish(True)

  def reset(self):
    self.reset.publish(True)

  def enter(self):
    if sys.version_info[0] < 3:
      raw_input("Press Enter to continue ")
    else:
      input("Press Enter to continue")

  ##launcher have to trigger here
  def learn(self):

    #Default value
    load = self.default_load()

    if load == 1:
      id = 0
      size = 20
      lin = 1.5
      ang = 1.5
      p_thr = 40
      r_thr = 180
      l_rate = 0.05
    else:
      id, size = self.aruco_marker()
      lin, ang = self.max_speed()
      p_thr,r_thr = self.obj_sensitivity()
      l_rate = self.l_rate()

    #Ask for a learning state
    state = self.state_cmd()

    print("Press enter to continue")

    #Press detection
    while(True):

      msg = '''
        Option: 
        1. Press I to Initiate
        2. Press S to Shutdown
        3. Press P to Pause
        4. Press C to continue
        (To do later) 
        5. Press R to Reset 
      '''
      print(msg)

      key = self.getKey()
      if key == 'i':
        print("Initiate the other nodes")
        #Initiate signal to all nodes
        self.init_pub.publish(True)

        #publish values after initiate first
        self.state_pub.publish(state)
        self.l_rate_pub.publish(l_rate)
        self.p_thr_pub.publish(p_thr)
        self.r_thr_pub.publish(r_thr)
        self.markerid_pub.publish(id)
        self.markersize_pub.publish(size)
        self.linmax_pub.publish(lin)
        self.angmax_pub.publish(ang)

        print("Press enter to trigger a reference point")
        self.enter()
        self.reference_pub.publish(True)

        print("Press enter to trigger a robot start signal")
        self.enter()
        self.move = True
        self.move_pub.publish(self.move)

        print("Press enter to trigger a robot stop signal")
        self.enter()
        self.move = False
        self.move_pub.publish(self.move)
        print("One second Cooldown")
        rospy.sleep(1)
        print('reset init to False state')
        self.init_pub.publish(False)

        print("Press enter back a menu section")
        self.enter()

      elif key == 's':
        print("shutdown all related nodes") 
        self.init_pub.publish(False)
        self.shutdown()
        rospy.signal_shutdown("Shutdown all nodes") 
        break

      elif key == 'p':
        print("Pause")
        self.init_pub.publish(False)
      
      elif key== 'c':
        print("Continue")
        self.init_pub.publish(True)

      #Still finding the way to reset nodes
      elif key == 'r':
        print("Reset all nodes")
        self.reset.publish(True)
        self.init_pub.publish(False)
        self.reset.publish(False)
        
  def getKey(self):
      #Launch termios setting
      settings = termios.tcgetattr(sys.stdin)
      tty.setraw(sys.stdin.fileno())
      select.select([sys.stdin], [], [], 0)
      key = sys.stdin.read(1)
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      return key

#Test area
if __name__ == "__main__":
  pass