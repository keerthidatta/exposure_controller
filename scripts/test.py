#!/usr/bin/env python
import math
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_control_msgs.msg import currentParams
from camera_control_msgs.srv import SetExposure

rospy.init_node('exposure_controller')
rospy.wait_for_service('stereo_camera/left/set_exposure')
exposure_set = rospy.ServiceProxy('stereo_camera/left/set_exposure', SetExposure)
reached_exposure = exposure_set(200000.0)

rospy.loginfo("new exposure value %f reached", reached_exposure.reached_exposure)
log_message = "Succesful" if reached_exposure.success else "Failed"
rospy.loginfo("configuration %s ", log_message)
  
