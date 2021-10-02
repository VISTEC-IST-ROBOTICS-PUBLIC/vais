import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from vais.msg import vais_param
from std_msgs.msg import Bool, Float32, String
from signals import ICO_Signal
from learn import Learning
import datetime
import sys
import time

class Core(object):

    def __init__(self):
        #Parameters    
        self.pos_dict = {}
        self.ref_dict = {}
        self.sig_dict = {}
        self.prev_dict = {}
        self.thr_list = []
        self.prev_time = -1
        self.state = ''
        self.prev_reflex = 0

        #Instantiation
        self.signal = ICO_Signal()
        self.learn = Learning()

        #Flags
        self.init = False

        #Pub topic
        self.ico_out = rospy.Publisher('/ico/output', Float32, queue_size = 1)

        #Sub topics
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        rospy.Subscriber('/signal/init', Bool, self.init_cb, queue_size =1)
        rospy.Subscriber('/data/vais_param', vais_param , self.vais_cb, queue_size=1)

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

    def init_cb(self, signal):                              #initial signal to trigger a reference capture                         
        self.init = signal.data

    def vais_cb(self, data):                                #read state from param
        self.state = data.state
        self.thr_list = [data.thr_list[0], data.thr_list[1], data.thr_list[2]]

    def main(self, time, pos_dict):
        #Use menu node to trigger a save of reference position
        self.ref_capture(pos_dict)

        if self.ref_dict:                            
            #Signal generation to product predictive, reflexive signal       
            self.signal_generation(self.ref_dict, self.pos_dict)

            if self.prev_time:
                diff = self.diff_time(time)
                date_time = datetime.datetime.fromtimestamp(time.to_sec())
                result = self.learn.ico(date_time, diff, self.state, self.sig_dict, self.prev_dict)

                #On screen for debug
                print(result)
                #publish to drive
                self.ico_out.publish(result)
 
            else:
                #print('Error: please check previous time step generation')
                pass
            #Copy to previous signal
            self.prev_dict=self.sig_dict.copy()

        else:
            #print('Error: please check reference position')
            pass

    def diff_time(self, time):
        #Later time step
        if self.prev_time != -1:
            diff_time = self.signal.truncated((time-self.prev_time).to_sec())
            #After finished, store current time as a previous time step for the next iteration
            self.prev_time = time
            return diff_time

        else:
            #First time step
            print("Initial state: First timestamp is logged")
            self.prev_time = time

    def ref_capture(self, pos_dict):
        #if no reference is stored, signal does not trigger
        if not self.ref_dict and self.init == True:
            self.ref_dict = pos_dict.copy()
            print ('Reference point: ', self.ref_dict)
        
        else:
            pass

    def signal_generation(self, ref, pos):  
        for key in ref: 
            #Case where the keys from both dicts are matched
            if key in pos:
                result = self.signal.obj_signal(ref[key], pos[key], self.thr_list)   #Use position list only.
                self.sig_dict[key] = result
                #Condition where the reflexive signal already reached 1, we would like to stop the robot.
                if self.prev_reflex == 1:
                    print("Reflex hits: Pause")
                    time.sleep(10)
                    sys.exit()               
                else:
                    #Condition where we need a previous reflex to calculate on a next time step.
                    self.prev_reflex = self.sig_dict[key][1]
                
            else:
                #This case means we already have the object on scene, but somehow disappeared from the screen at a current time.
                print("Error: current POS not found, please check the object (PREDICT:0, REFLEX: 1")
                self.sig_dict[key] = [0,1]
                time.sleep(10)
                sys.exit()

