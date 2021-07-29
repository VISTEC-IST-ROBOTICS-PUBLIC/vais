

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

        ##Signals
        rospy.subscriber('/signal/reference', Bool, self.reference_cb, queue_size=1)        #Reference point (triggered from main)
        rospy.subscriber('/signal/init', Bool, self.init_cb, queue_size=1)
        rospy.subscriber('/signal/shutdown', Bool, self.shutdown_cb, queue_size=1)


    #considering on 1. previous time 2. previous signal
    def ico(self, stamp):

            #load[-1] and load[-2] #think of 3 cases 1. no prev 2. only has -1 3. has both -1 and -2

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


###Test area -----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
  pass