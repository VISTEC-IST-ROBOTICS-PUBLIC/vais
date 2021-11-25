import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from vais.msg import vais_param
from std_msgs.msg import Bool, Float32, String
from signals import ICO_Signal
from learn import Learning
import datetime
import time
import math

class Core(object):

    def __init__(self):
        #Initial Parameters    
        self.ref_dict = {}
        self.sig_dict = {}
        self.prev_dict = {}
        self.thr_list = []
        self.prev_time = None
        self.l_rate = None
        self.ar_capture = False
        self.prev_reflex = 0
        self.move = False

        #Instantiation
        self.signal = ICO_Signal()
        self.learn = Learning()

        #Publishers
        self.ico_out = rospy.Publisher('/ico/output', Float32, queue_size = 1)
        self.ar_capture_pub = rospy.Publisher('/signal/ar_capture',Bool, queue_size = 1)
        self.shutdown_pub = rospy.Publisher('/signal/shutdown', Bool, queue_size=1)

        #Subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        rospy.Subscriber('/signal/ar_capture', Bool, self.ar_capture_cb, queue_size =1)
        rospy.Subscriber('/signal/shutdown', Bool, self.shutdown_cb, queue_size =1)
        rospy.Subscriber('/data/vais_param', vais_param , self.vais_cb, queue_size=1)
        rospy.Subscriber('/robot/move', Bool, self.move_cb, queue_size=1)

    def main(self, time, pos_dict):
        #Use menu node to manually capture an AR reference position
        self.ref_capture(pos_dict)
        if self.ref_dict:                            
            #Signal generation to product predictive, reflexive signal       
            self.signal_generation(self.ref_dict, pos_dict)

            if self.prev_time:
                diff = self.diff_time(time)
                #This human readable date time is used in the output log
                date_time = datetime.datetime.fromtimestamp(time.to_sec())
                result = self.learn.ico(date_time, diff, self.state, self.l_rate, self.sig_dict, self.prev_dict, self.move)

                #On screen for debug
                print("[INFO: RESULT]", result)
                #publish to drive
                self.ico_out.publish(result)
 
            else:
                #print("[Error]: please check previous time step generation")
                self.prev_time = time

            #Copy to previous signal
            self.prev_dict=self.sig_dict.copy()

        else:
            #print("[Error]: please check reference position")
            pass

    def diff_time(self, time):
        #Later time step
        if self.prev_time is not None:
            diff_time = self.signal.truncated((time-self.prev_time).to_sec())
            #After finished, store current time as a previous time step for the next iteration.
            self.prev_time = time
            return diff_time

        else:
            #First time step
            print("[INFO]: Initial state, first timestamp is logged")
            self.prev_time = time

    def ref_capture(self, pos_dict):
        if self.ar_capture == True:
            self.ref_dict = pos_dict.copy()
            print ("[INFO]: Reference point: ", self.ref_dict)
            self.ar_capture = False
            #Signal back to make ref_dict unrewritable
            self.ar_capture_pub.publish(self.ar_capture)
        
        else:
            #print("[ERROR]: Please check an AR capture signal")
            print ("[INFO]: Reference point: ", self.ref_dict)
            pass

    def signal_generation(self, ref, pos):  
        for key in ref: 
            #Case where the keys from both dicts are matched
            if key in pos:
                result = self.signal.obj_signal(ref[key], pos[key], self.thr_list)   #Use position list only.
                self.sig_dict[key] = result
                #print('obj: ', self.sig_dict[key][0])
                #print('pre: ', self.sig_dict[key][1])
                #print('ref:', self.sig_dict[key][2])
                #Condition where the reflexive signal already reached 1, we would like to stop the robot.
                if self.prev_reflex >= 1:
                    print("[INFO]: Reflex hits, Pause")
                    time.sleep(10)
                    self.shutdown_pub.publish(True)               
                else:
                    #Condition where we need a previous reflex to calculate on a next time step.
                    self.prev_reflex = self.sig_dict[key][2]
                    print("[INFO]: Diag previous reflex ", self.prev_reflex)
                
            else:
                #This case means we already have the object on scene, but somehow disappeared from the screen at a current time.
                print("[Error]: current POS ",key," is not found, please check the object (SET OBJ_DET:0, PREDICT:0, REFLEX: 1")
                self.sig_dict[key] = [0, 1, 1]
                print("[INFO]: Object lost, Pause")
                time.sleep(10)
                self.shutdown_pub.publish(True)

    def alvar_cb(self, alvar_pt):                                                                       #alvar always keep spinning its callback
        time = alvar_pt.header.stamp
        #This pos dict is a temporary dictionary to keep current position updated.
        pos_dict = {}

        if alvar_pt.markers:
            #ETL from msg to dict for each detected markers
            for index, value in enumerate(alvar_pt.markers):
                #Avoid nan value and unnecessary alvar_tag
                #if value.id < 1 or value.id > 17 or math.isnan(value.pose.pose.position.x) or math.isnan(value.pose.pose.position.y) or math.isnan(value.pose.pose.position.z):
                
                #This method is safer to specfied a single alvar_tag to avoid a ghosting tag
                if value.id != 14 or math.isnan(value.pose.pose.position.x) or math.isnan(value.pose.pose.position.y) or math.isnan(value.pose.pose.position.z): 
                    pass
                else:
                    pos_dict[value.id] = [value.pose.pose.position.x, value.pose.pose.position.y, value.pose.pose.position.z]
            self.main(time, pos_dict)
            #print("[INFO]: ALVAR #: ", value.id)
        else:
            #Case of no markers
            print ("[ERROR]: No alvar markers detected")
            pass

    def ar_capture_cb(self, signal):                                                                    #initial signal to trigger a reference capture                         
        self.ar_capture = signal.data

    def vais_cb(self, data):                                                                            #Read VAIS parameters from menu
        self.state = data.state
        self.l_rate = data.l_rate
        self.thr_list = [data.e_object, data.p_object, data.r_object]

    def shutdown_cb(self, signal):
        #Signal to shutdown this from input node.
        if signal.data == True:
            rospy.signal_shutdown("[INFO]: Shutdown signal is received, turn this node off")

    #Signal from input node to order the robot to move
    def move_cb(self, signal):
        self.move = signal.data