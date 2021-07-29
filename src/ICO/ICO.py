

class Ico(object):
    
    def __init__(self):
        pass

    def dist_predict(self):
        pass

    def dist_reflex(self):
        pass

    def obj_predict(self):
        pass

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