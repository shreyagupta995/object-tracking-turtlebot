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
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=3)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv_image = cv2.cvtColor(cv_image,cv.CV_BGR2GRAY)
            #cv_image = cv2.medianBlur(cv_image,3)
        except CvBridgeError as e:
            print(e)

        # construct the argument parse and parse the arguments
        # ap = argparse.ArgumentParser()
        # ap.add_argument("-i", "--image", help = "path to the image")
        # args = vars(ap.parse_args())

        # # load the image
        # image = cv2.imread(args["image"])

        # define the list of boundaries
        boundaries = [([86, 31, 4], [220, 88, 50])]

        # loop over the boundaries
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(cv_image, lower, upper)
            output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        # show the images
        # cv2.imshow("images", np.hstack([cv_image, output]))
        # cv2.waitKey(0)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)