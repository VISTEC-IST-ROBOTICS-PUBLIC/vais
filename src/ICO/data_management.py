import csv
import os
from os import path
import time
import sys

###File Save/Load section-------------------------------------------------------------------------------------------------------------
### Note: This data management method is designed to use on the Ubuntu system only
###CSV Format [Timestamp, weight, predict, reflex, derivative, euclidean]

class Data(object):
    
    def __init__(self):
        pass

    def filename(self, id , state):
        str_id = str(id)
        name = "ICOLOG_"+str_id+"_"+state+".csv"
        return name

    def filepath(self, filename):
        script_path = os.path.abspath(__file__)
        script_dir = os.path.split(script_path)[0]
        rel_path = 'data/'+filename
        filepath = os.path.join(script_dir, rel_path)
        return path

    def filecheck(self, filename):
        path = self.filepath(filename)
        try:
            if (os.path.exists(path) == True):
                print('File exists')
            else:
                print('Creates a new file')
                self.create(filename)
        except: 
            print("File check error, please check filename")
            time.sleep(30)
            sys.exit()


    def create(self, filename):
        try:
            #create file
            path = self.filepath(filename)
            with open(path,'w') as open_dat:
                #create header
                writer = csv.writer(open_dat)
                #writer.writerow(["Timestamp", "Weight", "Predictive", "Reflexive", "Derivative", "Euclidean"])       #Header (if needed)
                print("file created")
        except:
            print('Error: cannot create file')
            time.sleep(30)
            sys.exit()        

    def save(self, filename, stamp, weight, predict, reflex, derivative, euclidean):
        path = self.filepath(filename)
        try:
            with open(path,'a') as open_dat:
                writer = csv.writer(open_dat)
                writer.writerow([stamp, weight ,predict, reflex, derivative, euclidean])
                print("information saved")
        except:
            print('Error: cannot save file')
            time.sleep(30)
            sys.exit()

    def load(self, filename):
        path = self.filepath(filename)
        
        #modify this****
        with open(path, 'r') as f:
            last_line = f.readlines()[-1]
            try:
                #data available
                
                return sth
            else:
                return 0

        except:
            pass


###Test area-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
  pass