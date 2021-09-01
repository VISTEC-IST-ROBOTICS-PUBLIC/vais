import math
import numpy as np

class ICO_Signal(object):
    
    def __init__(self):
        #default initial threshold
        self.p_thr = 4
        self.r_thr = 8

    #Scale points from ALVAR (m to cm)
    def alvar_scale(self, point):       
        ##This conversion is converting into xx.yyy format
        new_scale = self.truncated(point * 100)
        return new_scale

    #Decimal points modification (3 points)
    def truncated(self, value):
        truncated = float('%.3f'%(value))
        return truncated

    def normalization(self, input, min, max):		
        nm = (input - min)/(max - min)
        return float(nm)

    #LPF (should be in ICO?)
    def LowPassFilter(self, sig, sig_prev):
        
        #Constants
        sig_factor = 0.7
        sig_prev_factor = 0.3

        new_sig = (sig_factor * sig) + (sig_prev_factor * sig_prev)

        return new_sig

    def euclidean_dist(self, ref_list, cur_list):
        #First, find the different
        diff_x = self.truncated(self.alvar_scale(ref_list[0])-self.alvar_scale(cur_list[0]))
        diff_y = self.truncated(self.alvar_scale(ref_list[1])-self.alvar_scale(cur_list[1]))
        diff_z = self.truncated(self.alvar_scale(ref_list[2])-self.alvar_scale(cur_list[2]))

        #Second, square them
        square_x = np.power(diff_x, 2)
        square_y = np.power(diff_y, 2)
        square_z = np.power(diff_z, 2)

        #Last, sum and square root
        euc_result = self.truncated(math.sqrt(square_x+square_y+square_z))
        return euc_result

    def dist_predict(self):
        pass

    def dist_reflex(self):
        pass

    def obj_reflex(self, ed):
        #format (id: stamp, p_signal, r_signal)    

        if ed < self.p_thr:
            return 0.0

        elif ed > self.p_thr and ed < self.r_thr:
            nm = self.normalization(ed, self.p_thr, self.r_thr)
            nm_trunc = self.truncated(nm)
            return nm_trunc

        else:
            return 1.0

    #main
    def obj_signal(self, ref_list, cur_list):
        euclidean = self.euclidean_dist(ref_list, cur_list)
        reflex = self.obj_reflex(euclidean)
        #predict is set to be always 1
        return [1, reflex]

    if __name__ == "__main__":
	    print ('dummy main')
