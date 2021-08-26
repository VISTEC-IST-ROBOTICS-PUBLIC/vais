import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool, Float32, String




class Core(object):

    def __init__(self):
        #Instantiation

        #Parameters    
        self.pos_dict = {}

        #Topics
        ##AR alvar markers (From alvar package)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_cb, queue_size=1)
        

    def alvar_cb(self, alvar_pt):                   #alvar will keep spinning its callback


		if alvar_pt.markers:
			#ETL from msg to dict
			for index, value in enumerate(alvar_pt.markers):
				#print (index)
				#print(value.id)
				#print(value.id, ': ', value.pose.pose.position.x)
				self.pos_dict[value.id] = [value.pose.pose.position.x, value.pose.pose.position.y, value.pose.pose.position.z]

			print self.pos_dict
		
		else:
			print ('null')



		#print (type(self.pos_dict[3][0]))
		#1. forward to signal
		#2. timestamp issue
		#3. check value dict
		#4. think about triggering things

		return self.pos_dict


    if __name__ == "__main__":
		print ('test')
	

