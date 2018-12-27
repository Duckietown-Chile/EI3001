 #!/usr/bin/env python
import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.srv import SetValue, SetValueResponse


class Listener(object):
	"""docstring for ClassName"""
	def __init__(self):
		super(Listener, self).__init__()
	    self.sub1 = rospy.Subscriber("/tenduckiebot/tag_detections", AprilTagDetectionArray, self.callback)
	    self.destination_server =  rospy.Service('/tenduckiebot/taxi', SetValue, self.get_destination)
		self.duckiebot_set_k = rospy.ServiceProxy('/tenduckiebot/inverse_kinematics_node/set_k', SetValue)
		self.duckiebot_set_k.wait_for_service()
		self.target_tag = 0

	def callback(data):
    	for detection in data.detections:
    		if detection.id == self.target_tag:
    			self.stop_duckiebot()
    			self.target_tag = 0
    			rospy.sleep(10)
    			self.go_duckiebot()


    def get_destination(req):
    	self.target = int(req.value)
    	return SetValueResponse()

    
	def stop_duckiebot():
		self.duckiebot_set_k(100)
		rospy.loginfo("Stopping for leaving or get a duckie!!")

	def go_duckiebot():
		self.duckiebot_set_k(27)
		rospy.loginfo("I'm on my Way!")

	
	    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    rospy.init_node('taxi', anonymous=False)
    Listener()
    rospy.spin()
