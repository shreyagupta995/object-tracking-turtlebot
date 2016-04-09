#!/usr/bin/env python

import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
from PIL import Image, ImageDraw
import roslib
import rospy
import tracking as tr
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys
roslib.load_manifest('bb8')

class BlueDetector:

    def __init__(self):
        rospy.init_node('BlueDetector', anonymous=True)
        self.image_pub = rospy.Publisher("bb8_tracking",Image,queue_size=3)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.count = 0

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

        # Threshold the BGR image to get only blue colors
        mask = cv2.inRange(cv_image, lower_blue, upper_blue)

        # Bitwise-AND mask and original image
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        #Publish to topic
        
        # List storing all the blue pixel positions
        rows, columns = [], []
        # Store the minimum and maximum column values for blue pixels
        min_row, max_row, min_col, max_col = 0, len(mask)-1, 0, len(mask[0])-1
        mid_row, mid_col = 0, 0
        flag = False
        
        
        # Frame resolution = 480x640
        # Store all pixel positions which fall within the bounds
        
        for row in range(max_row+1):
            for col in range(max_col+1):
                if mask[row][col]>0:
                    # if row > max_row:
                    #     max_row = row
                    # if row < min_row:
                    #     min_row = row
                    # if col > max_col:
                    #     max_col = col
                    # if col < min_col:
                    #     min_col = col
                    flag = True
                    rows.append(row)
                    columns.append(col)

        if flag == True:
            # Rough estimate of the center of the blue object - will help direct Turtlebot in the right direction
            mid_row = sum(rows)//len(rows) # min_row + (max_row-min_row)/2
            mid_col = sum(columns)//len(columns) # min_col + (max_col-min_col)/2
            cv2.circle(output,(mid_row,mid_col),20,(0,0,0),thickness=5,shift=2)
            #circle.ellipse([math.max(0,mid_row-20),math.max(0,mid_col-20)],[math.min(max_row,mid_row+20),math.min(max_col,mid_col+20)])

        print (mid_row, mid_col)
        #tracker = tr.TrackBB8((mid_row,mid_col), (len(mask),len(mask[0])))

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('BlueDetector', anonymous=True)
    ic = BlueDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)