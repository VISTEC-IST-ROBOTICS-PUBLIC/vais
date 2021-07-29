import math
import numpy as np

class Signal(object):
    


    def __init__():
        pass

    def pos(self, alvar_pt):
        #Retrieve each point from marker
        stamp = alvar_pt.markers.header.stamp
        id = alvar_pt.markers.id

        #Formatted to xx.yyy
        x = self.alvar_scale(alvar_pt.markers.pose.pose.position.x)
        y = self.alvar_scale(alvar_pt.markers.pose.pose.position.y)
        z = self.alvar_scale(alvar_pt.markers.pose.pose.position.z)

        return stamp, id, x, y, z

    #Scale points from ALVAR
    def alvar_scale(self, point):
        
        ##This conversion is converting into xx.yyy format
        new_scale = point * 100
        truncated = self.truncated(new_scale)
        return truncated

    #Decimal points modification (3 points)
    def truncated(self, value):
        truncated = float('%.3f'%(value))
        return truncated

    #Normalize the value
    def normalization(self, input, min, max):		
        nm = (input - min)/(max - min)
        return float(nm)

    #LPF
    def LowPassFilter(self, sig, sig_prev):
        
        #Constants
        sig_factor = 0.7
        sig_prev_factor = 0.3

        new_sig = (sig_factor * sig) + (sig_prev_factor * sig_prev)

        return new_sig

    def euclidean_dist(self, ref_x, ref_y, ref_z, cur_x, cur_y, cur_z):
        #First, find the different
        diff_x = ref_x-cur_x
        diff_y = ref_y-cur_y
        diff_z = ref_z-cur_z

        #Second, square them
        square_x = np.power(diff_x, 2)
        square_y = np.power(diff_y, 2)
        square_z = np.power(diff_z, 2)

        #Last, sum and square root
        euc_result = math.sqrt(square_x+square_y+square_z)

        #Truncated the result
        euc_result = self.truncated(euc_result)

        #Debugging purpose
        #print("Euclidean Distance: ", euc_result)
        return euc_result

###Signal section-------------------------------------------------------------------------

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
