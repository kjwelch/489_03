# -*- coding: utf-8 -*-
import rospy
from cv2 import destroyAllWindows
#from behaviourv3 import behaviour
import cv2
import numpy as np 
import rospy
from cv2 import startWindowThread, namedWindow
from numpy import mean
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError 
from geometry_msgs.msg import Twist
import time


class behaviour:

    @classmethod
    def __init__(self):
        rospy.loginfo("Running Behaviour...")
        namedWindow("Turtlebot View", 1)
        namedWindow("Green Isolation", 1)
        self.scansub = rospy.Subscriber("/turtlebot_1/scan", LaserScan, self.avoid)
        time.sleep(1) # Solves race condition error 
        self.imsub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)
        self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)
        self.bridge = CvBridge()
        startWindowThread()

    @classmethod
    def callback(self, data):
        # Handle the CvBridge exception in the event that the ROS image data is incorrectly processed
        #try:
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #except CvBridgeError, e:
        print(data)
        # Colour segmentation of the green channel
        bgr_thresh = cv2.inRange(self.cv_image, np.array((0, 0, 0)), np.array((0, 255, 0)))
        # Conversion from open CV's BGR format to the HSV format
        hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img, np.array((25, 50, 10)), np.array((100, 255, 255)))

        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in hsv_contours:
            a = cv2.contourArea(i)
            if a > 100.0:
                cv2.drawContours(self.cv_image, i, -1, (255, 0, 0), 2)

        cv2.imshow("Turtlebot View", self.cv_image)
        greenImg = cv2.bitwise_and(hsv_img,hsv_img,mask = hsv_thresh)

        cv2.imshow("Green Isolation", greenImg)
        im_left, im_right = np.hsplit(greenImg, 2)

        namedWindow("left", 1)
        cv2.imshow("left", im_left)
        namedWindow("right", 1)
        cv2.imshow("right", im_right)

        r_intensity = mean(im_right)
        l_intensity = mean(im_left)
        
        # Define located value from the comparison of the left and right  mean image intensity
        if l_intensity > r_intensity:
            self.fear(0)# No sliced object detected
        else: 
            if l_intensity < r_intensity:
                self.fear(1) # Sliced object detected
            else:
                self.fear(2) 
            
            
    @classmethod        
    def fear(self, vel):
        # Set frequency in Hz
        r = rospy.Rate(5) 
        rospy.loginfo("Fear Response")
        t = Twist()
        call_time = rospy.get_time()
        turn_dur = call_time + 20
        if (self.scan > 1): 
         
            if (vel == 0):
                if call_time < turn_dur:
                    print("Scared Search") 
                    t.angular.z = 0.5
                    time.sleep(1)
                    t.linear.x = 0.4
            else:
                #if green is in view, rotate
                if(vel == 1):
                    print("Turn and Run!")
                    if call_time < turn_dur:
                        t.angular.z = -3
                        t.linear.x = 0.1
                else: 
                # Wander through environment until next contact with 
                    print("Safe?")
                    t.linear.x = 0.2
                    t.angular.z = 0.1
                
        else: 
            # Ostacle avoidance velocity
            print("Avoiding Obstacle...")
            t.angular.z = -0.4;
            
        print(t)
        self.pub.publish(t) # Sending the command        
        r.sleep()
            
    @classmethod    
    def avoid(self, data):
        # Laser scan implementing minimum range obstacle avoidance
        self.scan = np.nanmin(data.ranges)

def main():
    rospy.init_node('wandering', anonymous=True)
    behaviour()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    destroyAllWindows()
   

if __name__ == "__main__":
    main()