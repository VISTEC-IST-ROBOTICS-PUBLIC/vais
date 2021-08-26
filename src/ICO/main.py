import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool, Float32, String
from signals import ICO_Signal

class Core(object):

    def __init__(self):

        #Parameters    
        self.pos_dict = {}
        self.ref_dict = {}
        self.sig_dict = {}

        #Instantiation
        self.signal = ICO_Signal()

        #Flags
        self.init = False

        #Topics
        ##AR alvar markers (From alvar package)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        rospy.Subscriber('/signal/init', Bool, self.init_cb, queue_size =1)

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
            print ('NULL: no alvar markers detected')

		#print (type(self.pos_dict[3][0]))
		#1. forward to signal
		#3. check value dict
		#4. think about triggering things

        return self.pos_dict

    def init_cb(self, signal):                              #initial signal to trigger a reference capture                         
        self.init = signal.data

    def main(self, time, pos_dict):
        
        print('Time: ', time)
        #time conversion
        #second = self.epoch_conversion(time)

        #initial state, robot stand still with multiple items in the tray.
        #reference trigger, this method captures all markers
        self.ref_capture(pos_dict)

        if self.ref_dict:                                   
            #checker
            print ('REF: ', self.ref_dict)                  #Reference point
            print ('POS: ', self.pos_dict)                  #Current point
            #self.signal_generation(ref, pos)
        else:
            print('Dropped: Not enough information (Please Check REF/POS')

    def signal_generation(self, ref, pos):
        for key in ref: 
            pass

            #Key match
            #if pos list disappeared reflex is 1 (out-of-sight)


    def ico(sig_dict):
        pass

    def ref_capture(self, pos_dict):
        #if no reference stored and capture signal does not trigger
        if not self.ref_dict and self.init == True:
            self.ref_dict = pos_dict.copy()
            print ('REF: ', self.ref_dict)
        
        else:
            pass


    #might be in ICO
    def epoch_conversion(self, epoch):

        sec = epoch
        return sec


    if __name__ == "__main__":
	    print ('dummy main')
	

