from data_management import Data
from signals import ICO_Signal

class Learning(object):
    
    def __init__(self):
        self.data = Data()
        self.signal = ICO_Signal()



    def ico(self, time, diff_time, state, sig_dict, prev_dict):

        #if key, load key with file
        #ICOLOG_ID_MOVEMENT.csv

        state = 'stand'

        if prev_dict:

            for key in sig_dict:
                if (key in sig_dict.keys()):
                    filename = self.data.filename(key, state)
                    self.data.filecheck(filename)
                    p_weight = self.data.load(filename)
                    print('P_WEIGHT:', p_weight)
                   # p_weight = 0.5

                    result = (sig_dict[key][0] * p_weight + sig_dict[key][1])
                    delta = self.signal.truncated(sig_dict[key][1] - prev_dict[key][1]*sig_dict[key][0]/diff_time)
                    new_weight = self.signal.truncated((p_weight+delta))
                    print(new_weight)
                    self.data.save(filename, time, new_weight, sig_dict[key][0], sig_dict[key][1], delta)
                else:
                    pass
        
        else:
            pass


        #Send a largest weight to drive

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