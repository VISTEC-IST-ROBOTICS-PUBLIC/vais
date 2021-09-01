from data_management import Data

class Learning(object):
    
    def __init__(self):
        self.data = Data()


#load
#prev check
#main
#save

    def ico(self, diff_time, state, sig_dict, prev_dict):

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