#!/usr/bin/env python
import rospy
#from anti_instagram.AntiInstagram import *
from anti_instagram.kmeans_rebuild import *
from anti_instagram.calcLstsqTransform import *
from anti_instagram.scale_and_shift import *
from cv_bridge import CvBridge  # @UnresolvedImport
# @UnresolvedImport
from duckietown_msgs.msg import (AntiInstagramHealth, AntiInstagramTransform,
                                 BoolStamped)
from duckietown_utils.jpg import image_cv_from_jpg
from line_detector.timekeeper import TimeKeeper
from sensor_msgs.msg import CompressedImage, Image  # @UnresolvedImport
import numpy as np


class AntiInstagramNode:
    def __init__(self):
        self.node_name = rospy.get_name()

        self.active = True
        self.locked = False

        self.image_pub_switch = rospy.get_param(
            "~publish_corrected_image", False)

        # Initialize publishers and subscribers
        self.pub_image = rospy.Publisher(
            "~corrected_image", Image, queue_size=1)
        self.pub_health = rospy.Publisher(
            "~health", AntiInstagramHealth, queue_size=1, latch=True)
        self.pub_transform = rospy.Publisher(
            "~transform", AntiInstagramTransform, queue_size=1, latch=True)

        #self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        #self.sub_image = rospy.Subscriber("~uncorrected_image",Image,self.cbNewImage,queue_size=1)
        self.sub_image = rospy.Subscriber(
            "~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)
        self.sub_click = rospy.Subscriber(
            "~click", BoolStamped, self.cbClick, queue_size=1)

        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', True)

        # Initialize health message
        self.health = AntiInstagramHealth()

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it

        # create instance of kMeans
        self.fancyGeom = rospy.get_param("~fancyGeom", "")
        self.n_centers = rospy.get_param("~n_centers", "")
        self.blur = rospy.get_param("~blur","")
        self.resize = rospy.get_param("~resize","")
        self.blur_kernel = rospy.get_param("~blur_kernel")
        self.KM = kMeansClass(self.n_centers, self.blur, self.resize, self.blur_kernel)

        self.corrected_image = Image()
        self.bridge = CvBridge()
        self.scale = np.array([1, 1, 1])
        self.shift = np.array([0, 0, 0])
        self.ai_health = 0.1

        self.image_msg = None
        self.click_on = False

    def cbNewImage(self, image_msg):
        # memorize image
        print("new image received.")
        self.image_msg = image_msg

        if True:
            tk = TimeKeeper(image_msg)
            # cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image = image_cv_from_jpg(self.image_msg.data)

            corrected_image_cv2 = scaleandshift2(cv_image, self.scale, self.shift)
            tk.completed('applyTransform')

            corrected_image_cv2 = np.clip(corrected_image_cv2, 0, 255).astype(np.uint8)
            self.corrected_image = self.bridge.cv2_to_imgmsg(corrected_image_cv2, "bgr8")

            tk.completed('encode')

            self.pub_image.publish(self.corrected_image)

            tk.completed('published')

            if self.verbose:
                rospy.loginfo('ai:\n' + tk.getall())

    def cbClick(self, _):
        # if we have seen an image:
        if self.image_msg is not None:
            self.click_on = not self.click_on
            if self.click_on:
                self.processImage(self.image_msg)
            else:
                self.transform.s = [0, 0, 0, 1, 1, 1]
                self.pub_transform.publish(self.transform)
                rospy.loginfo('ai: Color transform is turned OFF!')

    def processImage(self, msg):
        '''
        Inputs:
            msg - CompressedImage - uncorrected image from raspberry pi camera

        Uses anti_instagram library to adjust msg so that it looks like the same
        color temperature as a duckietown reference image. Calculates health of the node
        and publishes the corrected image and the health state. Health somehow corresponds
        to how good of a transformation it is.
        '''

        rospy.loginfo('ai: Computing color transform...')
        tk = TimeKeeper(msg)

        # cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        try:
            cv_image = image_cv_from_jpg(msg.data)
        except ValueError as e:
            rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
            return

        tk.completed('converted')

        # apply KMeans
        self.KM.applyKM(cv_image, fancyGeom=self.fancyGeom)

        # get the indices of the matched centers
        idxBlack, idxRed, idxYellow, idxWhite  = self.KM.determineColor(True, self.KM.trained_centers)

        # get centers with red
        trained_centers = np.array([self.KM.trained_centers[idxBlack], self.KM.trained_centers[idxRed],
                                self.KM.trained_centers[idxYellow], self.KM.trained_centers[idxWhite]])

        # get centers w/o red
        trained_centers_woRed = np.array([self.KM.trained_centers[idxBlack], self.KM.trained_centers[idxYellow],
                                self.KM.trained_centers[idxWhite]])

        # calculate transform with 4 centers
        T4 = calcTransform(4, trained_centers)
        T4.calcTransform()

        # calculate transform with 3 centers
        T3 = calcTransform(3, trained_centers_woRed)
        T3.calcTransform()

        # compare residuals
        # in practice, this is NOT a fair way to compare the residuals, 4 will almost always win out,
        # causing a serious red shift in any image that has only 3 colors
        if T4.returnResidualNorm() >= T3.returnResidualNorm():
            self.shift = T4.shift
            self.scale = T4.scale
        else:
            self.shift = T3.shift
            self.scale = T3.scale	

        self.shift = T3.shift
        self.scale = T3.scale
        tk.completed('calculateTransform')

        # if health is much below the threshold value, do not update the color correction and log it.
        if self.ai_health <= 0.001:
            # health is not good

            rospy.loginfo("Health is not good")

        else:
            self.health.J1 = self.ai_health
            self.transform.s[0], self.transform.s[1], self.transform.s[2] = self.shift
            self.transform.s[3], self.transform.s[4], self.transform.s[5] = self.scale

            self.pub_health.publish(self.health)
            self.pub_transform.publish(self.transform)
            rospy.loginfo('ai: Color transform published.')


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('anti_instagram_node', anonymous=False)

    # Create the NodeName object
    node = AntiInstagramNode()

    # Setup proper shutdown behavior
    # rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
