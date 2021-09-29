from data_management import Data
from signals import ICO_Signal

class Learning(object):
    
    def __init__(self):
        self.data = Data()
        self.signal = ICO_Signal()
        self.l_rate = 0.05
        self.res_dict = {}


    def ico(self, time, diff_time, state, sig_dict, prev_dict):
        if prev_dict:
            for key in sig_dict:
                if (key in sig_dict.keys()):
                    filename = self.data.filename(key, state)
                    self.data.filecheck(filename)
                    p_weight = self.data.load(filename)
                    result = (sig_dict[key][0] * p_weight + sig_dict[key][1])
                    delta = self.signal.truncated(self.l_rate*(sig_dict[key][1] - prev_dict[key][1])*sig_dict[key][0]/diff_time)
                    new_weight = self.signal.truncated((p_weight+delta))
                    self.data.save(filename, time, new_weight, sig_dict[key][0], sig_dict[key][1], delta)
                    self.res_dict[key] = result
                else:
                    pass
        
            #print("RESULT: ",self.res_dict)
            #find max to publish it to drive
            find_max = max(self.res_dict.keys(), key=lambda k:self.res_dict[k])
            max_value = self.res_dict[find_max]
            return max_value

        else:
            pass

