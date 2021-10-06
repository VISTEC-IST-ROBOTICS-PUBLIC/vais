#!/usr/bin/env python

import rospy
from output.base_output import MOVO_output
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from ICO.data_management import Data
from std_msgs.msg import Bool, Float32

class Demo1(object):
    def __init__(self):

        #Publishers
        self.odom_capture_pub = rospy.Publisher('/signal/odom_capture',Bool, queue_size = 1)
        self.move_pub = rospy.Publisher('/robot/move',Bool, queue_size = 1)
        self.ico_pub = rospy.Publisher('/ico/output', Float32, queue_size = 1)

        #Subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)

        #Pair dictionary [ID: ico]
        self.id_dict = {}

    # Move by meter unit
    def forward(self, target):
        op = MOVO_output()
        state = 'Angular'
        goal = [target,0,0]
        if self.ico_out:
            op.output_main(state, self.ico_out,goal)

    # + is a CCW rotaion, - is a CW rotation.
    def turn(self, target):
        op = MOVO_output()
        state = 'Angular'
        goal = [0,0,target]
        if self.ico_out:
            op.output_main(state, self.ico_out,goal)

    #Test input    
    def main(self):
        self.forward(0.5)
        self.turn(90)
        self.forward(1)
        self.turn(-90)

    #Return a maximum ico_output value
    def alvar_cb(self, alvar_pt):
        time = alvar_pt.header.stamp
        if alvar_pt.markers:
            for value in enumerate(alvar_pt.markers):
                if value.id < 17:
                    ico = self.load_ico(value.id)
                    self.id_dict[value.id] = [ico]

            find_max = max(self.id_dict.keys(), key=lambda k:self.id_dict[k])
            self.ico_out = self.id_dict[find_max]

        else:
            #print('Please check alvar callback')
            pass
           
    #Load last recent stored ico_output
    def load_ico(self, id):
        dat = Data()
        name = dat.filename(id)
        ico_output = dat.load_op(name)
        return ico_output

