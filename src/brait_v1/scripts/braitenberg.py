#!/usr/bin/env python3
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist



class image_converter:

  # instance variables
  def __init__(self):
    #self.image_pub = rospy.Publisher("new_image_pub",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/image",Image,self.callback)
    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      # Let's rotate the image by 180 degrees
      #cv_image = cv2.rotate(cv_image, cv2.cv2.ROTATE_180)
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    #divide image into thirds
    third = cols//3
    left_view = gray[:, :third]
    mid_view = gray[:, third:third*2]
    right_view = gray[:, third*2:]

    left_avg = np.average(left_view)
    right_avg = np.average(right_view)
    print(left_avg,right_avg)

    t = Twist()

    if left_avg > right_avg:
      print("left")
      t.angular.z = 0.1
      t.linear.x = 0.01
    elif right_avg > left_avg:
      print("right")
      t.angular.z = -0.1
      t.linear.x = 0.01
    
    self.pub.publish(t) # Sending the command        


    # Here's where the circle gets added
    # check out the api: 
    # https://www.geeksforgeeks.org/python-opencv-cv2-circle-method/
#    cv2.circle(cv_image, (50,50), 10, (0,0,255))

    #cv2.imshow("Image window", cv_image)
    cv2.imshow("Grey window", gray)
    cv2.imshow("left", left_view)
    cv2.imshow("right", right_view)
    cv2.waitKey(3)


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
