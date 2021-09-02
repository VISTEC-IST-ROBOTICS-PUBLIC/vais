import math
import numpy as np

class ICO_Signal(object):
    
    def __init__(self):
        #default initial threshold
        self.e_obj_thr = 3
        self.p_obj_thr = 6
        self.r_obj_thr = 9
        self.p_dist_thr = 0
        self.r_dist_thr = 1

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

    def dist_calculation(self):
        pass

    def obj_calculation(self, ed):        #format (id: stamp, p_signal, r_signal)    

        #case lies within exemption range
        if ed < self.e_obj_thr:
            return 0.0, 0.0

        #case of moving in a predictive area
        elif ed >= self.e_obj_thr and ed < self.p_obj_thr:
            nm = self.normalization(ed, self.e_obj_thr, self.p_obj_thr)
            nm_trunc = self.truncated(nm)
            return nm_trunc, 0.0

        #case of moving in a reflexive area
        elif ed > self.p_obj_thr and ed < self.r_obj_thr:
            nm = self.normalization(ed, self.p_obj_thr, self.r_obj_thr)
            nm_trunc = self.truncated(nm)
            return 1.0, nm_trunc

        #move beyond reflexive area
        else:
            return 1.0, 1.0

    #main
    def obj_signal(self, ref_list, cur_list):
        euclidean = self.euclidean_dist(ref_list, cur_list)
        predict, reflex = self.obj_calculation(euclidean)
        print("PREDICT: ", predict)
        print("REFLEX: ", reflex)
        return [predict, reflex]

    if __name__ == "__main__":
	    print ('dummy main')
