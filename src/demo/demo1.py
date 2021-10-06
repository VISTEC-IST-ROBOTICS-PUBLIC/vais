#!/usr/bin/env python

import rospy
from output.base_output import MOVO_output
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from ICO.data_management import Data

class Demo1(object):
    def __init__(self):
        #Sub topics
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)


    def main(self):
        op = MOVO_output()


        #Waypoint planner
        op.target('Linear', icoout)

    def load_ico(self):
        dat = Data()
        name = dat.filename()
        ico_w = dat.load_op(name)

    #Extract only ID
    def alvar_cb(self, alvar_pt):                           #alvar always keep spinning its callback
        time = alvar_pt.header.stamp

        if alvar_pt.markers:
            #ETL from msg to dict for each detected markers
            for index, value in enumerate(alvar_pt.markers):
                self.pos_dict[value.id] = [value.pose.pose.position.x, value.pose.pose.position.y, value.pose.pose.position.z]
            self.main(time, self.pos_dict)
        else:
            #case of no markers
            print ('NULL: no alvar markers detected')

"""draft
1. camera, pull value
2. trigger
3. check loop publish, output
4. finish and continue
5. finish line

"""