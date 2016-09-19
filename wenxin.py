#!/usr/bin/python
import rospy
import tf
import math
#~ from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt, transpose
from sympy import *

import numpy as np
import std_msgs.msg
from std_msgs.msg import String
#~ from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import nav_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
#~ from geometry_msgs.msg import PoseWithCovarianceStamped
#~ from geometry_msgs.msg import TwistWithCovarianceStamped
#~ from geometry_msgs.msg import TransformStamped

import geometry_msgs.msg
import threading




class ekf_class(object):

    def __init__(self):
        num_run = 0
        self.pub = rospy.Publisher('/epostip',std_msgs.msg.Int32, queue_size=1)
        while(1):
            for num in range(-180,180): 
                tangent = math.degrees(math.atan(math.cos(math.radians(num))))
                
                r = rospy.Rate(25)
                r.sleep()
                if tangent <0:
                    tangent += 360
                print tangent, num
                self.pub.publish(round(tangent))
                num_run += 1
                
            for num in range(180,-180): 
                tangent = math.degrees(math.atan(math.cos(math.radians(num))))
                
                r = rospy.Rate(25)
                r.sleep()
                if tangent <0:
                    tangent += 360
                print tangent, num
                self.pub.publish(round(tangent))    
            num_run += 1
            if num_run > 0:
                break

    
      
      

if __name__ == '__main__':
    rospy.init_node('ekf', anonymous=True)
    

    try:
        ekf = ekf_class()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
