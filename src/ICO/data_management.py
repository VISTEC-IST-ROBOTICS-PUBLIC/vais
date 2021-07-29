import csv
import os

###File Save/Load section-------------------------------------------------------------------------------------------------------------

class Data(object):
    
    def __init__(self):
        pass

    ###CSV Format
    ###Timestamp, predict, reflex, weight

    def filecheck(self):
        try:
            if (os.path.exists(self.abspath) == True):
                print('File exists')
            else:
                print('Creates a new file')
                self.create_csv()
        except: 
            print("File check error")

    def create_csv(self):
        try:
            #create file
            with open(self.abspath,'w') as open_dat:
                #create header
                writer = csv.writer(open_dat)
                #writer.writerow(["Timestamp", "Weight", "Predictive", "Reflexive", "Euclidean"])
                print("file created")
        except:
            print('Error: cannot create file')
        

    def save(self, stamp, weight, predict, reflex, euclidean):
        try:
            #create file
            with open(self.abspath,'a') as open_dat:
                #create header
                writer = csv.writer(open_dat)
                writer.writerow([stamp, weight ,predict, reflex, euclidean])
                print("information saved")
        except:
            print('Error: cannot save file')

    def load(self, id, state):
        str_id = str(id)
        filename = "ICOLOG_"+str_id+"_"+state+".csv"
        file_path = path.relpath("data/"+filename)
        with open(file_path, 'r') as f:
            if
            #lines = f.read().splitlines()
            #last_line = lines[-1]
            last_line = f.readlines()[-1]
            return last_line
        #load if no append

###Test area-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
  pass