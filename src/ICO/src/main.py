import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool, Float32, String

from ICO import signal


class main(object):

    def __init__(self):
        self.signal =signal        
        #Instantiation

        #Parameters    
        self.pos_dict = {}

        #Topics
        ##AR alvar markers (From alvar package)
        rospy.subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        

    def alvar_cb(self, alvar_pt):                   #alvar will keep spinning its callback
        return alvar_pt


    def main(self, alvar):


        
        #ref first

        #loop?



        #First get stamp [id1, [xyz1] id2, [xyz2]]
        pass

