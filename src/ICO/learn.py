from data_management import Data
from signals import ICO_Signal

class Learning(object):
    
    def __init__(self):
        self.data = Data()
        self.signal = ICO_Signal()
        self.res_dict = {}              #result of each object in dictionary

    def ico(self, time, diff_time, state, l_rate , sig_dict, prev_dict):
        if prev_dict:
            #Retrieve each object ID in dictionary
            for key in sig_dict:
                if (key in sig_dict.keys()):
                    filename = self.data.filename(key, state)
                    self.data.filecheck(filename)
                    p_weight = self.data.load(filename)
                    result = (sig_dict[key][0] * p_weight + sig_dict[key][1])
                    delta = self.signal.truncated(l_rate*(sig_dict[key][1] - prev_dict[key][1])*sig_dict[key][0]/diff_time)
                    new_weight = self.signal.truncated((p_weight+delta))
                    self.data.save(filename, time, new_weight, sig_dict[key][0], sig_dict[key][1], delta)
                    self.res_dict[key] = result
                else:
                    #print('Cannot obtain any object ID from signal dictionary')
                    pass
        
            #print("RESULT: ",self.res_dict)
            #find maximum possible result to publish it to drive. More value on result means slower robot drive
            find_max = max(self.res_dict.keys(), key=lambda k:self.res_dict[k])
            max_value = self.res_dict[find_max]
            return max_value

        else:
            #Previous signal is needed on the later steps
            #print('First time step')
            pass

