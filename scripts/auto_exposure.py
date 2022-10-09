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

bridge = CvBridge()
current_exposure_time = None
optimal_brightness = 2.5 #grey card 18 percent
error_i = 0
right_image_brightness = 0.0
left_image_brightness = 0.0
DEFAULT_KP = 0.05
DEFAULT_KI = 0.01
MAX_I = 3
ERROR_THRESHOLD = 0.5

def get_image_brightness(image):
    
    rows, columns, channels = image.shape
    if (channels == 3):
	brightness_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:,:,2]
    else:
        brightness_image = image
    hist = cv2.calcHist([brightness_image], [0], None, [5], [0,256])
    mean_sample_value = 0

    #Adapted from "Automatic Camera Exposure Control" by N. Nourani-Vatani, J.Roberts
    for i in range(len(hist)):
	mean_sample_value += hist[i]*(i+1)

    mean_sample_value /= rows*columns
    return mean_sample_value

def set_exposure(exposure_time, service):
    rospy.wait_for_service(service)
    set_exposure_value = rospy.ServiceProxy(service, SetExposure)
    reached_exposure = set_exposure_value(exposure_time)
    
    rospy.loginfo("new exposure value %f reached for %s", reached_exposure.reached_exposure, service)
    log_message = "Succesful" if reached_exposure.success else "Failed"
    rospy.loginfo("configuration %s for %s", log_message, service)

def controller(image_brightness, service):
    # assert:
    if abs(left_image_brightness - right_image_brightness) > 1.0:
   	rospy.logwarn("Brightness of left and right images are different: automatic exposure will not be performed. Check settings of left and right cameras")
    else:  
        rospy.loginfo("Exposure controller running without any issues")  
	global error_i
	error_p = optimal_brightness - image_brightness
	error_i += error_p

	if abs(error_i) > MAX_I:
	    error_i = np.sign(error_i) * MAX_I

        #Control only when error is more than threshold: Camera frames per second drops if updated very often
	if abs(error_p) > ERROR_THRESHOLD:
	    rospy.loginfo("Error term %f", kp*error_p+ki*error_i)
	    control_value =  optimal_brightness + (kp*error_p + ki*error_i) 
            #derived relationship between exposure time and brightness from auto exposure algorithm 
	    required_exposure = current_exposure_time * (control_value/optimal_brightness)
	    set_exposure(required_exposure, service)
  
def right_image_callback(right_image):
    global right_image_brightness
    cv_image = bridge.imgmsg_to_cv2(right_image, "bgr8")
    right_image_brightness = get_image_brightness(cv_image)
    controller(right_image_brightness, right_exposure_service)

def left_image_callback(left_image):
    global left_image_brightness
    cv_image = bridge.imgmsg_to_cv2(left_image, "bgr8")
    left_image_brightness = get_image_brightness(cv_image)
    controller(left_image_brightness, left_exposure_service)

def get_exposure_time(exposure_time):
    global current_exposure_time
    current_exposure_time = exposure_time.exposure
    #print("exposure_time: ", current_exposure_time)

def main():
    rospy.init_node("exposure_controller")
    rospy.Subscriber(rospy.get_param("RIGHT_IMAGE_TOPIC", '/stereo_camera/right/image_raw'), Image, right_image_callback)
    rospy.Subscriber(rospy.get_param("LEFT_IMAGE_TOPIC", '/stereo_camera/left/image_raw'), Image, left_image_callback)
    rospy.Subscriber(rospy.get_param("STEREO_CURRENT_PARAMS_TOPIC", '/pylon_camera_node/currentParams'), currentParams, get_exposure_time)
    global left_exposure_service
    global right_exposure_service
    global kp, ki

    left_exposure_service = rospy.get_param("LEFT_EXPOSURE_SERVICE", 'stereo_camera/left/set_exposure')
    right_exposure_service = rospy.get_param("RIGHT_EXPOSURE_SERVICE", 'stereo_camera/right/set_exposure')
    kp = rospy.get_param("KP", DEFAULT_KP)
    ki = rospy.get_param("KI", DEFAULT_KI)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

