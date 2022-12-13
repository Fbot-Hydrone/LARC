#! /usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
import cv2
import time, os, math
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
import pyqrcode
# from PIL import Image
from pyzbar.pyzbar import decode

flag = True
pub = rospy.Publisher('/block_detector/detection', String, queue_size=10)
bridge_ = CvBridge()

basea = False
baseb = False
basec = False
based = False
basee = False

def activate_block_detection(data):
    global flag
    if data.data == True:
        print("Detection Enabled!")
        flag = True
    else:
        print("Detection Disabled!")
        flag = False

def image_callback(data):
    global flag

    global basea
    global baseb
    global basec
    global based
    global basee

    if flag == True:
        cvim = bridge_.imgmsg_to_cv2(data, "bgr8")
        
        data = decode(cvim)
        if len(data) != 0: 
            # rospy.loginfo("Detected value: %s", str(data[0].data))
            if data[0].data == "A" and basea == False:
                pub.publish("GREEN")  
                time.sleep(1.0)  
                pub.publish(data[0].data)
                basea = True
                rospy.loginfo("BASE A")
            if data[0].data == "B" and baseb == False:
                pub.publish("GREEN")   
                time.sleep(1.0)   
                pub.publish(data[0].data)
                baseb = True
                rospy.loginfo("BASE B")
            if data[0].data == "C" and basec == False:
                pub.publish("GREEN")  
                time.sleep(1.0)    
                pub.publish(data[0].data)
                basec = True
                rospy.loginfo("BASE C")
            if data[0].data == "D" and based == False:
                pub.publish("GREEN")    
                time.sleep(1.0)  
                pub.publish(data[0].data)
                based = True
                rospy.loginfo("BASE D")
            if data[0].data == "E" and basee == False:
                pub.publish("GREEN")  
                time.sleep(1.0)    
                pub.publish(data[0].data)
                basee = True
                rospy.loginfo("E")

if __name__ == "__main__": 
    rospy.init_node("gas_detector", anonymous=False)    

    rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, image_callback, queue_size = 1)
    # rospy.Subscriber("/usb_cam/image_raw", Image, image_callback, queue_size = 1)
    rospy.Subscriber("/block_detector/activate", Bool, activate_block_detection)
    
    rospy.spin()
