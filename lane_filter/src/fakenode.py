#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped
import rospy

class Fake(object):

    def __init__(self):
        self.node_name = "fake"
        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()
        self.publisher = rospy.Publisher("/lane_filter_node/car_cmd", Twist2DStamped, queue_size=1)

    def publish(self):
    	self.velocity.v = 0.1
    	self.velocity.omega = 0
    	self.velocity.header.stamp = rospy.Time.now()
    	self.publisher.publish(self.velocity)
if __name__ == '__main__':
    rospy.init_node('fake', anonymous=False)
    fake = Fake()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	fake.publish()
    	rate.sleep()
