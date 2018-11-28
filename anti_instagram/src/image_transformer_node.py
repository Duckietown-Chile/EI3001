#!/usr/bin/env python
import rospy
from anti_instagram.AntiInstagram_rebuild import *
from cv_bridge import CvBridge  # @UnresolvedImport
from duckietown_msgs.msg import (AntiInstagramHealth, AntiInstagramTransform, AntiInstagramTransform_CB, BoolStamped)
from duckietown_utils.jpg import bgr_from_jpg
from line_detector.timekeeper import TimeKeeper
from sensor_msgs.msg import CompressedImage, Image  # @UnresolvedImport
import numpy as np
import time
import threading
import cv2

"""
This node subscribed to the uncorrected images from the camera and corrects these images
    with the latest stored transform.
It also subscribed to the trafo message coming from the Anti-Instagram node. When received,
    it updates the stored transform parameters.
"""

class ImageTransformerNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        # Initialize transform messages

        self.transform = AntiInstagramTransform()
        self.transform_CB = AntiInstagramTransform_CB()

        # initialize AI class
        self.ai = AntiInstagram()
        # initialize image bridge
        self.bridge = CvBridge()
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', False)


        self.active = True
        self.locked = False
        self.thread_lock = threading.Lock()
        self.r = rospy.Rate(5) # Rate in Hz
        self.scale_percent=40
        rospy.set_param("~scale_percent", self.scale_percent)

        robot_name = rospy.get_param("~veh", "") #to read the name always reliably
        # Read parameters
        
        self.trafo_mode = self.setupParameter("~trafo_mode", 'both')
        if not (self.trafo_mode == "cb" or self.trafo_mode == "lin" or self.trafo_mode == "both"):
            rospy.loginfo("cannot understand argument 'trafo_mode'. set to 'both' ")
            self.trafo_mode == "both"
            rospy.set_param("~trafo_mode", "both")  # Write to parameter server for transparancy
            rospy.loginfo("[%s] %s = %s " % (self.node_name, "~trafo_mode", "both"))

        # Initialize publishers and subscribers
        self.pub_image = rospy.Publisher(
            "~corrected_image/compressed", CompressedImage, queue_size=1)

        self.sub_image = rospy.Subscriber(
            '/{}/camera_node/image/compressed'.format(robot_name), CompressedImage, self.callbackImage, queue_size=1)

        self.sub_trafo = rospy.Subscriber(
            '/{}/cont_anti_instagram_node/transform'.format(robot_name), AntiInstagramTransform, self.cbNewTrafo, queue_size=1)

        self.sub_colorBalance = rospy.Subscriber(
            '/{}/cont_anti_instagram_node/colorBalanceTrafo'.format(robot_name), AntiInstagramTransform_CB, self.cbNewTrafo_CB, queue_size=1)



        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', False)



        rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):
        self.scale_percent = rospy.get_param("~scale_percent")

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)#Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def callbackImage(self, image_msg):
        #use this to avoid lag!
        thread = threading.Thread(target=self.cbNewImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def cbNewImage(self, image_msg):
        # this callback proceeds the latest image, i.e. it applies the current transformation to the image and publishes it
        # begin2=rospy.Time.now()
        if not self.thread_lock.acquire(False):
                return

        # memorize image
        self.image_msg = image_msg

        tk = TimeKeeper(image_msg)
        try:
            cv_image = bgr_from_jpg(image_msg.data)
        except ValueError as e:
            rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
            return

        #cv_image = cv2.imread(cv_image)
        scale_percent=self.scale_percent
        width=int(cv_image.shape[1]*scale_percent / 100)
        height=int(cv_image.shape[0]*scale_percent / 100)
        dim = (width, height)
        cv_image=cv2.resize(cv_image,dim,interpolation=cv2.INTER_NEAREST )
        tk = TimeKeeper(image_msg)
        #
        # end0=rospy.Time.now()
        # duration0=end0-begin2
        # rospy.loginfo('Conversion: %s' % duration0)
        tk.completed('converted')



        if self.trafo_mode == "cb" or self.trafo_mode == "both":
            # apply color balance using latest thresholds
            # begin1=rospy.Time.now()
            colorBalanced_image_cv2 = self.ai.applyColorBalance(img=cv_image, ThLow=self.ai.ThLow, ThHi=self.ai.ThHi)
            tk.completed('applyColorBalance')
            # end1=rospy.Time.now()
            # duration=end1-begin1

            # rospy.loginfo('Complete CB-Trafo Duration: %s' % duration)
        else:
            # pass input image
            colorBalanced_image_cv2 = cv_image

        if self.trafo_mode == "lin" or self.trafo_mode == "both":
            # apply color Transform using latest parameters
            corrected_image_cv2 = self.ai.applyTransform(colorBalanced_image_cv2)
            tk.completed('applyTransform')
        else:
            # pass input image
            corrected_image_cv2 = colorBalanced_image_cv2
        # begin3=rospy.Time.now()
        # store image to ros message
        self.corrected_image = self.bridge.cv2_to_compressed_imgmsg(
            corrected_image_cv2)# , "bgr8")
        tk.completed('encode')

        self.corrected_image.header.stamp = image_msg.header.stamp  # for synchronization

        # publish image
        self.pub_image.publish(self.corrected_image)
        tk.completed('published')
        # end3=rospy.Time.now()
        # duration3=end3-begin3
        # rospy.loginfo('Publishing time: %s' % duration3)
        # begin4=rospy.Time.now()

        if self.verbose:
            rospy.loginfo('ai:\n' + tk.getall())

        #self.r.sleep() #to keep the rate
        self.thread_lock.release()
        # end4=rospy.Time.now()
        # duration4=end4-begin4
        # rospy.loginfo('END OF FUNCTION: %s' % duration4)
        # end2=rospy.Time.now()
        # duration2=end2-begin2
        # rospy.loginfo('TRANSFORMATION: %s' % duration2)

    def cbNewTrafo(self, trafo_msg):
        # this callback stores the received linear transformation parameters
        if self.verbose:
            rospy.loginfo('image transformer: received new trafo!')

        # memorize transform message
        self.transform = trafo_msg

        # store transform to the Anti-Instagram instance
        self.ai.shift = trafo_msg.s[0:3]
        self.ai.scale = trafo_msg.s[3:6]


    def cbNewTrafo_CB(self, th_msg):
        # this callback stores the received color balance transformation parameters
        if self.verbose:
            rospy.loginfo('image transformer: received new Color Balance trafo!')

        self.transform_CB = th_msg
        self.ai.ThLow = th_msg.th[0:3]
        self.ai.ThHi = th_msg.th[3:6]



if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('image_transformer_node', anonymous=False)

    # Create the NodeName object
    node = ImageTransformerNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
rospy.spin()
