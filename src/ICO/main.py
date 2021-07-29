import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool, Float32, String


class main(object):

    def __init__(self):
        
        #Instantiation

        #Parameters    
        self.pos_dict = {}

        #Topics
        ##AR alvar markers (From alvar package)
        rospy.subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        

    def alvar_cb(self, alvar_pt):                   #alvar will keep spinning its callback
        #for each alvar_pt
        #store in list? list = alvar method?
        #think about stamp, id, x, y, z management

        pass

    def main():

        #ref first

        #loop?



        #First get stamp [id1, [xyz1] id2, [xyz2]]
        pass

