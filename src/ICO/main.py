import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool, Float32, String
from signals import ICO_Signal
from learn import Learning

class Core(object):

    def __init__(self):
        #Parameters    
        self.pos_dict = {}
        self.ref_dict = {}
        self.sig_dict = {}
        self.prev_dict = {}
        self.prev_time = -1
        self.state = ''

        #Instantiation
        self.signal = ICO_Signal()
        self.learn = Learning()

        #Flags
        self.init = False

        #Topics
        ##AR alvar markers (From alvar package)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        rospy.Subscriber('/signal/init', Bool, self.init_cb, queue_size =1)
        rospy.Subscriber('/robot/state', String, self.state_cb, queue_size=1)

    def alvar_cb(self, alvar_pt):                           #alvar always keep spinning its callback
        time = alvar_pt.header.stamp

        if alvar_pt.markers:
            #ETL from msg to dict
            for index, value in enumerate(alvar_pt.markers):
                #print (index)
                #print(value.id)
                #print(value.id, ': ', value.pose.pose.position.x)
                self.pos_dict[value.id] = [value.pose.pose.position.x, value.pose.pose.position.y, value.pose.pose.position.z]
            self.main(time, self.pos_dict)
        else:
            #case of no markers
            print ('NULL: no alvar markers detected')
        return self.pos_dict

    def init_cb(self, signal):                              #initial signal to trigger a reference capture                         
        self.init = signal.data
    def state_cb(self, signal):                              #initial signal to trigger a reference capture                         
        self.signal = signal.data

    def main(self, time, pos_dict):
        #initial state, robot satetand still with multiple items in the tray.
        #Captures all markers as a reference position
        self.ref_capture(pos_dict)

        if self.ref_dict:                                   
            #checker
            #print ('REF: ', self.ref_dict)                  #Reference point
            #print ('POS: ', self.pos_dict)                  #Current point
            self.signal_generation(self.ref_dict, self.pos_dict)

            if self.diff_time(time):
                print('Notice: ICO')
                diff_time = self.diff_time(time)
                self.learn.ico(diff_time, self.state, self.sig_dict, self.prev_dict)

            else:
                pass
            #Copy to previous signal
            self.prev_dict=self.sig_dict.copy()

        else:
            print('Signal Dropped: Not enough information (Please Check REF/POS')

    def diff_time(self, time):
        if self.prev_time != -1:
            diff_time = self.signal.truncated((time-self.prev_time).to_sec())
            self.prev_time = time
            return diff_time

        else:
            #initial state
            print("Initial state: First log:, time/signal")
            self.prev_time = time
            return -1

    def ref_capture(self, pos_dict):
        #if no reference stored and capture signal does not trigger
        if not self.ref_dict and self.init == True:
            self.ref_dict = pos_dict.copy()
            print ('REF: ', self.ref_dict)
        
        else:
            pass

    def signal_generation(self, ref, pos):  
        for key in ref: 
            if key in pos:
                result = self.signal.obj_signal(ref[key], pos[key])   #Use position list only
                self.sig_dict[key] = result
            else:
                print("Error: current POS not found, please check the object (PREDICT:0, REFLEX: 1")
                self.sig_dict[key] = [0,1]
        
        #Debugging purpose
        #print(self.sig_dict)

    if __name__ == "__main__":
	    print ('dummy main')
	

