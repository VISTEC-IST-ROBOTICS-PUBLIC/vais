from data_management import Data
from signals import ICO_Signal

class Learning(object):
    
    def __init__(self):
        self.data = Data()              #Instantiation from data management module.
        self.signal = ICO_Signal()      #Instantiation from signal module. 
        self.res_dict = {}              #Store results in Python dict.

    def ico(self, time, diff_time, state, l_rate , sig_dict, prev_dict, move):
        if prev_dict:
            #Retrieve each object ID in dictionary
            for key in sig_dict:
                if (key in sig_dict.keys()):
                    filename = self.data.filename(key, state)
                    self.data.filecheck(filename)
                    p_weight = self.data.load(filename)
                    #Object D. + Predict + Reflex
                    result = (p_weight * sig_dict[key][0] + p_weight * sig_dict[key][1] * p_weight + sig_dict[key][2])
                    delta = self.signal.truncated(l_rate*(sig_dict[key][2] - prev_dict[key][2])*sig_dict[key][1]/diff_time)
                    #To avoid an object bouncing back and forth which causes the robot to rapidly changes (by suddenly increasing and decreasing) it speed back and forth
                    if delta < 0.0:
                        delta = 0
                    new_weight = self.signal.truncated((p_weight+delta))
                    #Only log value when the robot moves
                    if move == True:
                        self.data.save(filename, time, new_weight, sig_dict[key][0], sig_dict[key][1], sig_dict[key][2], delta, result)
                    self.res_dict[key] = result
                else:
                    print("[ERROR]: Cannot obtain any object ID from signal dictionary")
                    pass
        
            #print("RESULT: ",self.res_dict)
            find_max = max(self.res_dict.keys(), key=lambda k:self.res_dict[k])                         #Use winner takes all (most sensitve values are the chosen result to drive the robot)
            max_value = self.res_dict[find_max]
            return max_value

        else:
            #Previous signal is needed on the later steps
            #print("[INFO]: This is a first time step")
            pass

