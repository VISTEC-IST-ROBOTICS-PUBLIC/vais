import math
import numpy as np

class ICO_Signal(object):
    
    def __init__(self):
        pass

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

    def obj_calculation(self, ed, thr_list):          

        #threshold for exemption, predictive, reflexive area
        e_thr = thr_list[0]
        p_thr = thr_list[1]
        r_thr = thr_list[2]

        #case lies within exemption range
        if ed < e_thr:
            return 0.0, 0.0

        #case of moving in a predictive area
        elif ed >= e_thr and ed < p_thr:
            nm = self.normalization(ed, e_thr, p_thr)
            nm_trunc = self.truncated(nm)
            return nm_trunc, 0.0

        #case of moving in a reflexive area
        elif ed >= p_thr and ed < r_thr:
            nm = self.normalization(ed, p_thr, r_thr)
            nm_trunc = self.truncated(nm)
            return 1.0, nm_trunc

        #move beyond reflexive area
        else:
            return 1.0, 1.0

    #main
    def obj_signal(self, ref_list, cur_list, thr_list):
        euclidean = self.euclidean_dist(ref_list, cur_list)
        predict, reflex = self.obj_calculation(euclidean, thr_list)
        #print("PREDICT Signal: ", predict)
        #print("REFLEX Signal: ", reflex)
        return [predict, reflex]
