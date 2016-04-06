#!/usr/bin/env python

import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys
roslib.load_manifest('bb8')

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("bb8",Image,queue_size=3)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = cv2.cvtColor(cv_image,cv.CV_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        # define the list of boundaries
        # Modified BGR blue [([102, 51, 0], [255, 153, 51])]
        # Original BGR blue [([86, 31, 4], [220, 88, 50])]

        # HSV blue
        lower_blue = np.array([102, 51, 0]) # Orginal HSV lower:  Modified HSV lower: [210,100,40]
        upper_blue = np.array([255, 153, 51]) # Original HSV upper:  Modified HSV upper: [21,80,100]

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(cv_image, lower_blue, upper_blue)

        # Bitwise-AND mask and original image
        # find the colors within the specified boundaries and apply the mask
        output = cv2.bitwise_and(cv_image,cv_image, mask=mask)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)