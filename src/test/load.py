#!/usr/bin/env python

import rospy
from ICO.data_management import Data

class Load_test(object):
    def __init__(self):
        self.data_load = Data()

    def main(self,state, id):
        print("ID",id)
        result = self.data_load.load_result(state,id)
        print (result)

if __name__ == "__main__":
    Load_instance = Load_test()
    #Test ID_1 from result
    Load_instance.main("Angular", 1)
    Load_instance.main("Linear", 1)