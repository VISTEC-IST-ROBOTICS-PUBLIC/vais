import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from vais.msg import vais_param
from std_msgs.msg import Bool, Float32, String
import datetime
import time
import math
import numpy as np

class Euclidean_Test(object):

    def __init__(self):

        self.ref_dict = {}
        self.ar_capture = False

        #Subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        rospy.Subscriber('/signal/ar_capture', Bool, self.ar_capture_cb, queue_size =1)


    def alvar_cb(self, alvar_pt):  
        time = alvar_pt.header.stamp
        pos_dict = {}                                                                     #alvar always keep spinning its callback
        if alvar_pt.markers:
            #ETL from msg to dict for each detected markers
            for index, value in enumerate(alvar_pt.markers):
                #Avoid nan value and unnecessary alvar_tag
                #if value.id < 1 or value.id > 17 or math.isnan(value.pose.pose.position.x) or math.isnan(value.pose.pose.position.y) or math.isnan(value.pose.pose.position.z):
                
                #This method is safer to specfied a single alvar_tag to avoid a ghosting tag
                if math.isnan(value.pose.pose.position.x) or math.isnan(value.pose.pose.position.y) or math.isnan(value.pose.pose.position.z): 
                    pass
                else:
                    pos_dict[value.id] = [value.pose.pose.position.x, value.pose.pose.position.y, value.pose.pose.position.z]
            print(pos_dict)
            self.main(time, pos_dict)
            #print("[INFO]: ALVAR #: ", value.id)
        else:
            #Case of no markers
            print ("[ERROR]: No alvar markers detected")
            self.pos_dict = {}
            pass

    def ref_capture(self, pos_dict):
        if self.ar_capture == True:
            self.ref_dict = pos_dict.copy()
            print ("[INFO]: Reference point: ", self.ref_dict)
            self.ar_capture = False
            #Signal back to make ref_dict unrewritable
            #self.ar_capture_pub.publish(self.ar_capture)
        
        else:
            #print("[ERROR]: Please check an AR capture signal")
            print ("[INFO]: Reference point: ", self.ref_dict)
            pass

    def euclidean_dist(self, ref_list, cur_list):
        diff_x = self.truncated(self.alvar_scale(ref_list[0])-self.alvar_scale(cur_list[0]))
        diff_y = self.truncated(self.alvar_scale(ref_list[1])-self.alvar_scale(cur_list[1]))
        diff_z = self.truncated(self.alvar_scale(ref_list[2])-self.alvar_scale(cur_list[2]))

        square_x = np.power(diff_x, 2)
        square_y = np.power(diff_y, 2)
        square_z = np.power(diff_z, 2)

        euc_result = self.truncated(math.sqrt(square_x+square_y+square_z))
        print("[INFO]: EUCLIDEAN DISTANCE: ", euc_result)
        return euc_result

    def signal_generation(self, ref, pos):  
        for key in ref: 
            #Case where the keys from both dicts are matched
            if key in pos:
                print('key', key)
                print('pos', pos)
                self.euclidean_dist(ref[key],pos[key])
                
            else:
                if pos:
                    #This case means we already have the object on scene, but somehow disappeared from the screen at a current time.
                    print("[ERROR]: current POS ", key ," is not found")
                else:
                    print("[ERROR]: No marker detected")


    def main(self, time, pos_dict):
        #Use menu node to manually capture an AR reference position
        self.ref_capture(pos_dict)
        if self.ref_dict:                            
            #Signal generation to product predictive, reflexive signal       
            self.signal_generation(self.ref_dict, pos_dict)
        else:
            pass
            #print("[ERROR]: ref dict not found")


    def ar_capture_cb(self, signal):                                                                    #initial signal to trigger a reference capture                         
        self.ar_capture = signal.data

    
    #Decimal points modification (3 points)
    def truncated(self, value):
        truncated = float('%.3f'%(value))
        return truncated

    #Scale points from ALVAR (m to cm)
    def alvar_scale(self, point):       
        ##This conversion is converting into xx.yyy format
        new_scale = self.truncated(point * 100)
        return new_scale