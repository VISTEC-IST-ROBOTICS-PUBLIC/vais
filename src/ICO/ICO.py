import rospy
import math
import numpy as np
from std_msgs.msg import Bool, Float32, String

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers

import os
import csv

class Ico(object):
    
    def __init__(self):
        
    #Parameters
        ##Dictionary storage
        self.ref_dict = {}
        self.pos_dict = {}
        self.euc_dict = {}
        self.signal_dict = {}
        self.w_predict = {}

        ##Condition expression
        self.capture_ref = False

        ##Distance deviation threshold (cm)
        self.p_thr = 4                      #Acceptance radius
        self.r_thr = 8                      #Reflex limit radius

    #Topics
        ##Threshold (Threshold can be injected and change)
        rospy.Subscriber('/data/p_thr', Float32, self.pthr_cb, queue_size=1)
        rospy.Subscriber('/data/r_thr', Float32, self.rthr_cb, queue_size=1)

        ##AR alvar markers (From alvar package)
        rospy.subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)

        ##Signals
        rospy.subscriber('/signal/reference', Bool, self.reference_cb, queue_size=1)        #Reference point (triggered from main)
        rospy.subscriber('/signal/init', Bool, self.init_cb, queue_size=1)
        rospy.subscriber('/signal/shutdown', Bool, self.shutdown_cb, queue_size=1)
        rospy.subscriber('/robot/state', String, self.state_cb)
###Focused methods section-----------------------------------------------------------------------------------------------------------


### Signal generation section--------------------------------------------------------------------------------------------------------
    #Call back to retrieve QR position from ALVAR.
    def alvar_cb(self, alvar_pt):
    
        #Check message first (CHECK)

        ##Loop to retrieve all marker points (need to look at this)
        for m in alvar_pt:
            
            #Retrieve each point from marker
            stamp = alvar_pt.markers[m].header.stamp
            id = alvar_pt.markers[m].id

            #Formatted to xx.yyy
            x = self.alvar_scale(alvar_pt.markers[m].pose.pose.position.x)
            y = self.alvar_scale(alvar_pt.markers[m].pose.pose.position.y)
            z = self.alvar_scale(alvar_pt.markers[m].pose.pose.position.z)

            #store every markers information to pos_dict with a format {id1: [x1,y1,z1], id2: [x2,y2,z2], .. } 
            self.pos_dict[id] = [x,y,z]
            
        #throw to signal method after every markers are stored
        self.signal(stamp)

    #Method to capture reference point
    def capture_reference(self):
    
        if (self.capture_reference == False):
            print("Waiting for reference signal to be triggered")

            # if this id is already in this dictionary, passed
            if (id in self.ref_dict.keys()):
                #print("Reference point is already captured")
                pass
    
            else:
                self.ref_dict = self.pos_dict         #Copy pos_dict to ref_dict
                self.capture_reference = True       #Alter this signal to skip storing reference point

                #Debugging purpose
                #print("Reference point of ", id ," is: ", self.ref_dict[id][0],", ", self.ref_dict[id][1],", ", self.ref_dict[id][2])

        else:
            #print("Reference point is already captured")
            pass

    #Euclidean can be generated to use in two conditions
        ##first, distance avoidance condition
        ##second, object reached condition
    
    def generate_euclidean(self, stamp):

        #Store reference point
        self.capture_reference()

        #For every key stored in reference, if the key is found in pos_dict
        for key in self.ref_dict:
            if (key in self.pos_dict.keys()):
                result = self.euclidean_dist(self.ref_dict[key][0], self.ref_dict[key][1], self.ref_dict[key][2], self.pos_dict[key][0], self.pos_dict[key][1], self.pos_dict[key][2])
                self.generate_signal(key, stamp, result)

            else:
                print(key, " is lost in tracking")
        
        #Throw to ICO signal
        self.ico(stamp)

    ##make two signals generation because we need different condition
        ##first, distance avoidance condition
        ##second, object reached condition

    def dist_predict(self):
        pass

    def dist_reflex(self):
        pass

    def obj_predict(self):
        for key in self.signal_dict:
            if (key in self.signal_dict.keys()):
                return 1.0
            else:
                return 0.0

    def obj_reflex(self, key, stamp, ed):
        #format (id: stamp, p_signal, r_signal)    

        if ed < self.p_thr:
            return 0.0

        elif ed > self.p_thr and ed < self.r_thr:
            nm = self.normalization(ed, self.p_thr, self.r_thr)
            nm_trunc = self.truncated(nm)
            predict = 1.0
            return nm_trunc
                
        elif ed > self.r_thr:
            return 1.0

        else:
            return 1.0

        self.signal_dict[key] = [stamp, predict, reflex]

###ICO section----------------------------------------------------------------------------------------------------------------------
    #considering on 1. previous time 2. previous signal
    def ico(self, stamp):

        #if key, load key with file
        #ICOLOG_ID_MOVEMENT.csv

        for key in self.signal_dict:
            if (key in self.signal_dict.keys()):
                self.load(key)

                delta = float(self.signal_dict[key][2] - self.r_prev)

                pass
            else:
                pass

        #if first time trigger, load weight (from CSV)
        #if not, use self.weight
        #last append (to CSV)
        #This one will be published

    ## public term
        #sum(w_predict*predict) 
        #highest (w_reflex*reflex)
        ##result = predict + reflex

###Calculation section--------------------------------------------------------------------------------------------------------------
    def euclidean_dist(self, ref_x, ref_y, ref_z, cur_x, cur_y, cur_z):
        #First, find the different
        diff_x = ref_x-cur_x
        diff_y = ref_y-cur_y
        diff_z = ref_z-cur_z

        #Second, square them
        square_x = np.power(diff_x, 2)
        square_y = np.power(diff_y, 2)
        square_z = np.power(diff_z, 2)

        #Last, sum and square root
        euc_result = math.sqrt(square_x+square_y+square_z)

        #Truncated the result
        euc_result = self.truncated(euc_result)

        #Debugging purpose
        print("Euclidean Distance: ", euc_result)
        return euc_result

###Value modification section--------------------------------------------------------------------------------------------------------

    #Scale points from ALVAR
    def alvar_scale(self, point):
        
        ##This conversion is converting into xx.yyy format
        new_scale = point * 100
        truncated = self.truncated(new_scale)
        return truncated

    #Decimal points modification (3 points)
    def truncated(self, value):
        truncated = float('%.3f'%(value))
        return truncated

    #Normalize the value
    def normalization(self, input, min, max):		
        nm = (input - min)/(max - min)
        return float(nm)

    #LPF
    def LowPassFilter(self, sig, sig_prev):
        
        #Constants
        sig_factor = 0.7
        sig_prev_factor = 0.3

        new_sig = (sig_factor * sig) + (sig_prev_factor * sig_prev)

        return new_sig

###Signal call back section----------------------------------------------------------------------------------------------------------
    def init_cb(self, signal):
        self.init = signal.data

    def shutdown_cb(self, signal):
        self.init = signal.data
    
    def pthr_cb(self, signal):
        self.p_thr = signal.data
    
    def rthr_cb(self, signal):
        self.r_thr = signal.data

    def reference_cb(self, signal):
        self.capture_ref = signal.data

    def state_cb(self, state):
        self.state = state.data


###Test area -----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
  pass