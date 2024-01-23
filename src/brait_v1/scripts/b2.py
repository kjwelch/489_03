#!/usr/bin/env python3
#from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist



class vehicle:

  # instance variables
  def __init__(self):
    #self.image_pub = rospy.Publisher("new_image_pub",Image,queue_size=10)
    self.prev_gray = np.zeros((240,320), dtype=np.uint8)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/image",Image,self.callback)
    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

  def background_subtractor(self,gray):
   # fg_mask = cv2.createBackgroundSubtractorMOG2().apply(gray)
    fg_mask = cv2.createBackgroundSubtractorMOG2().apply(gray)
    fg_mask = cv2.adaptiveThreshold(fg_mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    return fg_mask

  def get_contour_detections(self,mask, thresh=400):
    """ Obtains initial proposed detections from contours discoverd on the mask. 
        Scores are taken as the bbox area, larger is higher.
        Inputs:
            mask - thresholded image mask
            thresh - threshold for contour size
        Outputs:
            detectons - array of proposed detection bounding boxes and scores [[x1,y1,x2,y2,s]]
        """
    # get mask contours
    contours, _ = cv2.findContours(mask, 
                                   cv2.RETR_EXTERNAL, # cv2.RETR_TREE, 
                                   cv2.CHAIN_APPROX_TC89_L1)
    detections = []
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        area = w*h
        if area > thresh: 
            detections.append([x,y,x+w,y+h, area])

    return np.array(detections)

  def get_mask(self,frame1, frame2, kernel=np.array((9,9), dtype=np.uint8)):
    """ Obtains image mask
        Inputs: 
            frame1 - Grayscale frame at time t
            frame2 - Grayscale frame at time t + 1
            kernel - (NxN) array for Morphological Operations
        Outputs: 
            mask - Thresholded mask for moving pixels
        """
    print("frame 1: ",frame1)
    print("type f1", type(frame1))
    print("frame 2: ",frame2)
    print("type f2", type(frame2))


    frame_diff = cv2.subtract(frame2, frame1)

    # blur the frame difference
    frame_diff = cv2.medianBlur(frame_diff, 3)
    
    mask = cv2.adaptiveThreshold(frame_diff, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY_INV, 11, 3)

    mask = cv2.medianBlur(mask, 3)

    # morphological operations
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    return mask
  #def compute_iou
  def callback(self,data):
   
    try:

      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # flip 180
      # only on turtlebot3_05S
      print("flipping")
      # cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      print("type:", type(gray))
      # print(gray.shape)
      mask = self.get_mask(self.prev_gray,gray)
      contour = self.get_contour_detections(mask, thresh=400)
      print(contour)
      rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # bboxes = contour[:, :4]
      # scores = contour[:, -1]

      
      for box in contour:
        x1,y1,x2,y2 = box[0],box[1],box[2],box[3]
        cv2.rectangle(rgb_mask, (x1,y1), (x2,y2), (255,0,0), 3)



      self.prev_gray = gray
      # fg_mask = fg_mask.apply(cv_image)
      fg_mask = self.background_subtractor(gray)
      
      # Let's rotate the image by 180 degrees
      
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

    movement_threshold = 150
    if left_avg > movement_threshold or right_avg > movement_threshold:
      print("left avg: ", left_avg)
      print("right avg: ", right_avg)
     


      print("Movement Detected!")

    llw = 0
    lrw = 2
    rrw = 0
    rlw = 2

    t = self.behave(left_avg,right_avg,llw,lrw,rrw,rlw) 
    self.pub.publish(t) # Sending the command        

    # Here's where the circle gets added
    # check out the api: 
    # https://www.geeksforgeeks.org/python-opencv-cv2-circle-method/
#    cv2.circle(cv_image, (50,50), 10, (0,0,255))

    #cv2.imshow("Image window", cv_image)
    cv2.imshow("Grey window", gray)
    cv2.imshow("left", left_view)
    cv2.imshow("right", right_view)
    cv2.imshow("mask",mask)
    # cv2.imshow("contour",contour)
    cv2.imshow("detected moves",rgb_mask)
    cv2.waitKey(1)
  
  def behave(self,left_intensity,right_intensity, l_l_w, l_r_w, r_r_w, r_l_w):
    '''
    given intensities of left and right sides of the vehicle as
    well as connection "weights" beween each sensor and each wheel, 
    make robot move
    '''
    #speed_factor = 0.01
    left_speed = left_intensity * l_l_w + right_intensity * r_l_w
    right_speed = right_intensity * r_r_w + left_intensity * l_r_w

    linear_coeff= 0.0001 
    angular_coeff= .005
    linear_speed = linear_coeff*(left_speed + right_speed)
    angular_speed = angular_coeff*(right_speed-left_speed)

    t = Twist()

    #t.angular.z = angular_speed
    #t.linear.x = linear_speed 

    t.angular.z = 0
    t.linear.x = 0


#    print("hello")
    print("linear:", linear_speed, "angular:",angular_speed)
    ## v = (R/2) * (vr + vl)
    ## w = (R/L) (vr - vl) 
    return t



def main(args):
  ic = vehicle()
  rospy.init_node('vehicle', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
