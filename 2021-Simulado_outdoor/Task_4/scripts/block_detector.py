#! /usr/bin/env python3
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

flag = False
pub = rospy.Publisher('/block_detector/detection', String, queue_size=10)
pub_number = rospy.Publisher("/block_detector/detection_number", Int8, queue_size=10)
pub_qrcode_distance = rospy.Publisher("/block_detector/dx_dy", Int16MultiArray, queue_size=1)
pub_found_qrcode = rospy.Publisher("/block_detector/found_qrcode", Bool, queue_size=10)
bridge_ = CvBridge()

basea = False
baseb = False
basec = False
based = False
basee = False

array_distance = Int16MultiArray()
array_distance.data = []

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

        mask = cv2.inRange(cvim,(0,0,0),(200,200,200))
        cvim = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        data = decode(cvim)


        rospy.loginfo("Detecting qrcode...")

        if len(data) != 0:
            pub_found_qrcode.publish(True)

            (x, y, w, h) = data[0].rect
            cv2.rectangle(cvim, (x, y), (x + w, y + h), (0, 0, 255), 2)

            imgHeight, imgWidth, _ = cvim.shape

            centerImgX = int(imgWidth/2)
            centerImgY = int(imgHeight/2 + imgHeight/10)

            centerQrCodeX = int((x+w) - (w/2))
            centerQrCodeY = int((y+h) - (h/2))

            cv2.circle(cvim, (centerImgX, centerImgY), 5, (255, 0, 0), 3)
            # cv2.circle(cvim, (centerQrCodeX, centerQrCodeY), 10, (0, 0, 255), 3)

            # rospy.loginfo("IMAGE CENTER (%s, %s)" % (centerImgX, centerImgY))

            distanceX = centerQrCodeX - centerImgX
            distanceY = centerQrCodeY - centerImgY

            # rospy.logwarn("Distance X: %s   Y: %s" % (distanceX, distanceY))

            array_distance.data = [distanceX , distanceY]
            pub_qrcode_distance.publish(array_distance)

            # rospy.loginfo("Detected value: %s", str(data[0].data))
            if data[0].data == "A" and basea == False:
                # pub.publish("GREEN")
                time.sleep(1.0)
                pub.publish(String(data=data[0].data))
                pub_number.publish(1)
                basea = True
                rospy.loginfo("BASE A")
            if data[0].data == "B" and baseb == False:
                # pub.publish("GREEN")
                time.sleep(1.0)
                pub.publish(String(data=data[0].data))
                pub_number.publish(2)
                baseb = True
                rospy.loginfo("BASE B")
            if data[0].data == "C" and basec == False:
                # pub.publish("GREEN")
                time.sleep(1.0)
                pub.publish(String(data=data[0].data))
                pub_number.publish(3)
                basec = True
                rospy.loginfo("BASE C")
            if data[0].data == "D" and based == False:
                # pub.publish("GREEN")
                time.sleep(1.0)
                pub.publish(String(data=data[0].data))
                pub_number.publish(4)
                based = True
                rospy.loginfo("BASE D")
            if data[0].data == "E" and basee == False:
                # pub.publish("GREEN")
                time.sleep(1.0)
                pub.publish(String(data=data[0].data))
                pub_number.publish(5)
                basee = True
                rospy.loginfo("E")


        cv2.imshow("detection_qrcode", cvim)

        cv2.waitKey(20)

if __name__ == "__main__":
    rospy.init_node("gas_detector", anonymous=False)

    rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, image_callback, queue_size = 1)
    rospy.Subscriber("/block_detector/activate", Bool, activate_block_detection)

    rospy.spin()
