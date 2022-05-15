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

    #Constructs a file name by reading the AR tag from camera.
    def filename(self, id , state):
        str_id = str(id)
        name = "ICOLOG_"+str_id+"_"+state+".csv"
        return name

    #Generates full filepath.
    def filepath(self, filename):
        script_path = os.path.abspath(__file__)
        script_dir = os.path.split(script_path)[0]
        rel_path = "data/"+filename
        path = os.path.join(script_dir, rel_path)
        return path

    #Check if file exists
    def filecheck(self, filename):
        path = self.filepath(filename)
        print(path)
        try:
            if (os.path.exists(path) == True):
                print("[INFO]: File exists")
            else:
                print("[INFO]: Creates a new file")
                self.create(filename)
        except: 
            print("[ERROR]: File check error, please check filename")
            time.sleep(30)
            sys.exit()

    #Create CSV file
    def create(self, filename):
        try:
            #create file
            path = self.filepath(filename)
            with open(path,'w') as open_dat:
                #create header
                writer = csv.writer(open_dat)
                #Header
                writer.writerow(["Timestamp", "Weight", "Object", "Predictive", "Reflexive", "Derivative", "ico_output", "Feedback_x", "Feedback_z"])
                print("[INFO]: File: "+filename+" has been created")
        except:
            print("[ERROR]: Cannot create: ", filename)
            time.sleep(30)
            sys.exit()        

    #Save information to the target file.
    def save(self, filename, stamp, weight, object, predict, reflex, derivative, ico_out, feedback_x, feedback_z):
        path = self.filepath(filename)
        try:
            with open(path,'a') as open_dat:
                writer = csv.writer(open_dat)
                writer.writerow([stamp, weight, object ,predict, reflex, derivative, ico_out, feedback_x, feedback_z])
                print("[INFO]: Information has been saved")
        except:
            print("[ERROR]]: Cannot save: ", filename)
            time.sleep(30)
            sys.exit()

    #Load weight information from the target file.
    def load(self, filename):
        path = self.filepath(filename)
        try:        
            with open(path, 'r') as f:
                try:
                    last_line = f.readlines()[-1]
                    line = last_line.split(',')
                    weight = float(line[1])
                    return weight
                except:
                    print("[INFO]: No trace of previous weight, return 0.0")
                    return 0.0

        except:
            print("[ERRROR]: Cannot load weight from: ", filename)
            time.sleep(30)
            sys.exit()

    
    #Weight load on result file
    def load_result(self, state, id):
        #result file is in a result sub folder (quick solution)
        result_file = "result/"+state+".csv"
        path = self.filepath(result_file)
        print(path)
        try:        
            with open(path, 'r') as f:
                try:
                    target_line = id
                    last_line = f.readlines()[target_line]
                    line = last_line.split(',')
                    #Read Average element
                    weight = float(line[7])
                    return weight
                except:
                    print("[ERROR]: Result weight does not exist, Please check a result file")
                    time.sleep(30)
                    sys.exit()

        except: 
            print("[ERRROR]: Cannot load weight from: ", result_file)
            time.sleep(30)
            sys.exit()

    def feedback_check(self, path):
        try:
            if (os.path.exists(path) == True):
                print("[INFO]: File exists")
            else:
                print("[INFO]: Creates a new file")
                self.feedback_create(path)
        except: 
            print("[ERROR]: File check error, please check filename")
            time.sleep(30)
            sys.exit()
        
    def feedback_create(self, path):
        print(path)
        try:
            with open(path,'w') as open_dat:
                #create header
                writer = csv.writer(open_dat)
                #Header
                writer.writerow(["Timestamp", "ico_out", "Vel_X", "Vel_Y", "Ang_Z"])
                print("[INFO]: File has been created")
        except:
            print("[ERROR]: Cannot create file")
            time.sleep(30)
            sys.exit()       

    def save_feedback(self, name, timestamp, ico_out, vel_x, vel_y, ang_z):
        feedback_file = "feedback/"+name+".csv"
        path = self.filepath(feedback_file)
        #print(path)
        self.feedback_check(path)

        try:
            with open(path, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, ico_out, vel_x, vel_y, ang_z])
                print("[INFO]: Feedback has been logged")
        except:
                print("[ERROR]: File cannot be logged")

    

