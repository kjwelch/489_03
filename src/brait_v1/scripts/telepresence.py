#!/usr/bin/env python3
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
#import antler_tracker as QTM
#import qtm
import math

def SetAngularSpeedBasedOnYawDifference(desired,actual):
    '''
    Given a desired yaw (of the marker cluster) and an actual yaw (of the robot), provide an
    angular speed that will rotate the robot to align with the marker.

    '''
    error = desired-actual
    if abs(error) > math.pi:
        if desired > actual:
           actual += 2*math.pi
        else:
            desired +=2*math.pi
        error =  desired - actual 
    return error

def SetLinearSpeedBasedOnHeight(arm_height):
    '''
    Given the height of the marker arm, provide a motor speed.  Heights above 1m are positive, below 1m are negative
    '''
    outspeed = arm_height - 1.0
    return outspeed

def SetAngularSpeedBasedOnArmRoll(armRoll):
   '''
   Given the roll of the marker cluster, provide an angular speed. Roll can be positive or negative.
   '''
   return armRoll 

def SetLinearSpeedBasedOnArmPitch(armPitch):
   '''
   Given pitch of a marker cluster, provide a linear speed based upon that pitch.  Can be positive or negative.
   '''
   return armPitch

def SetLinearSpeedBasedOnClusterSpeed(clusterSpeed):
  '''
  Given speed of cluster, provide a linear speed based on that speed.
  '''
  return clusterSpeed * 150

class vehicle:

  def __init__(self):

    self.odom_sub_arm = rospy.Subscriber("qualisys_node/ArmCluster4/odom",Odometry,self.callbackArm)
    self.odom_sub_robot = rospy.Subscriber("qualisys_node/Turtlebot5/odom",Odometry,self.callbackRobot)
    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    self.armYaw = 0
    self.armRoll = 0
    self.armPitch = 0
    self.armX = None
    self.armY = None
    self.armZ = None
    self.LastPosition = None



  def callbackArm(self,data):
     '''
     This function is called whenever we receive mocap data about the arm cluster
     '''
     #transform quaternion into roll/pitch/yaw
     (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
     print("Arm Position: ", data.pose.pose.position)
     print("Arm Orientation: ", roll, pitch, yaw)

     # set variables for use by other callbacks
     self.armYaw = yaw
     self.armRoll = roll
     self.armPitch = pitch
     self.armX = data.pose.pose.position.x
     self.armY = data.pose.pose.position.y
     self.armZ = data.pose.pose.position.z
     

  def callbackRobot(self,data):
     '''
     This function is called whenever we get data about the robot position
     '''

     #touch these at your own risk.
     angular_coefficient = 0.6
     linear_coefficient = 0.2
     (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
   
     print("Robot Position: ", data.pose.pose.position)
     print("Robot Orientation: ", roll, pitch, yaw) 
     t = Twist()
        
     # These control angular speed
     #desiredAngularSpeed = SetAngularSpeedBasedOnYawDifference(self.armYaw,yaw)
     desiredAngularSpeed = SetAngularSpeedBasedOnArmRoll(self.armRoll)

     # These control linear speed
     #desiredLinearSpeed = SetLinearSpeedBasedOnHeight(self.armZ)
     #desiredLinearSpeed = SetLinearSpeedBasedOnArmPitch(self.armPitch)

     desiredLinearSpeed = 0
     if self.LastPosition != None:
        print('arm Difference: ', self.armX - self.LastPosition)
        desiredLinearSpeed = SetLinearSpeedBasedOnClusterSpeed(self.armX - self.LastPosition)


     self.LastPosition = self.armX
  

     #set the speeds and send to robot
     t.angular.z = angular_coefficient*desiredAngularSpeed
     t.linear.x = linear_coefficient*desiredLinearSpeed
     self.pub.publish(t) # Sending the command

     print("linear:", desiredLinearSpeed , "angular:", desiredAngularSpeed)
        

  



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
