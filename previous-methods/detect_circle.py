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
      cv_image = cv2.cvtColor(cv_image,cv.CV_BGR2GRAY)
      cv_image = cv2.medianBlur(cv_image,3)
    except CvBridgeError as e:
      print(e)

    (rows,cols) = cv_image.shape
    if cols > 60 and rows > 60 :
      #cv2.circle(cv_image, (50,50), 10, 255)
      circles = cv2.HoughCircles(cv_image,cv.CV_HOUGH_GRADIENT,1,1000,param1=50,param2=30,minRadius=0,maxRadius=0)
      circles = np.uint16(np.around(circles))
      for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
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
