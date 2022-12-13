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
first_detection = True
start_detection = False
qrcode_value = ""

pub = rospy.Publisher('/block_detector/detection', String, queue_size=10)
pub_number = rospy.Publisher("/block_detector/detection_number", Int8, queue_size=10)
pub_qrcode_distance = rospy.Publisher("/block_detector/dx_dy", Int16MultiArray, queue_size=1)
pub_found_qrcode = rospy.Publisher("/block_detector/found_qrcode", Bool, queue_size=10)
bridge_ = CvBridge()


array_distance = Int16MultiArray()
array_distance.data = []

qrcodes_detected = []


QRCODE_WHITELIST = ["A", "B", "C", "D", "E"]

DICT_LETTERS_NUMBERS = {
    "A": 1,
    "B": 2,
    "C": 3,
    "D": 4,
    "E": 5
}


def activate_block_detection(data):
    global flag
    if data.data == True:
        print("Detection Enabled!")
        flag = True
    else:
        print("Detection Disabled!")
        flag = False


def getQRcodeDistances(img, qrcode):
    (x, y, w, h) = qrcode.rect
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

    imgHeight, imgWidth, _ = img.shape

    centerImgX = int(imgWidth/2)
    centerImgY = int(imgHeight/2 - imgHeight/4)

    centerQrCodeX = int((x+w) - (w/2))
    centerQrCodeY = int((y+h) - (h/2))

    cv2.circle(img, (centerImgX, centerImgY), 3, (255, 0, 0), 3)
    cv2.circle(img, (centerQrCodeX, centerQrCodeY), 3, (0, 0, 255), 3)

    # rospy.loginfo("IMAGE CENTER (%s, %s)" % (centerImgX, centerImgY))

    distanceX = centerQrCodeX - centerImgX
    distanceY = centerQrCodeY - centerImgY

    return distanceX, distanceY



def image_callback(callback_img):
    global flag
    global first_detection
    global qrcode_value
    global start_detection
    global qrcodes_detected

    if flag == True:

        img = bridge_.imgmsg_to_cv2(callback_img, "bgr8")

        qrcodes = decode(img)

        # detector = cv2.QRCodeDetector()
        # data, bbox, straight_qrcode = detector.detectAndDecode(img)
        if len(qrcodes) != 0:

            for qrcode in qrcodes:

                if str(qrcode.data)[2] in QRCODE_WHITELIST:

                    # print(f"QRCode data:\n{str(qrcode.data)[2]}")

                    if not(str(qrcode.data)[2] in qrcodes_detected):
                        pub_found_qrcode.publish(True)
                        qrcode_number = DICT_LETTERS_NUMBERS[ str(qrcode.data)[2] ]

                        pub.publish(String(data=str(qrcode.data)[2]))
                        pub_number.publish(qrcode_number)

                        dx, dy = getQRcodeDistances(img, qrcode)

                        array_distance.data = [dx , dy]
                        pub_qrcode_distance.publish(array_distance)

                        if first_detection == True:
                            # qrcodes_detected.append(qrcode.data)
                            start_detection = True
                            qrcode_value = str(qrcode.data)[2]
                            print("\nQrcode detectado. Valor: {}\n".format(str(qrcode.data)[2]))

                            first_detection = False


                # if qrcode.data in QRCODE_WHITELIST:
                #     if first_detection == True:

                #         if qrcode.data in qrcodes_detected:
                #             rospy.logwarn("QRCODE %s JA DETECTADO" % qrcode.data)
                #             continue
                #         else:
                #             try:
                #                 print(qrcode.data)
                #                 qrcode_number = DICT_LETTERS_NUMBERS[ qrcode.data ]

                #                 pub.publish(String(data=qrcode.data))
                #                 pub_number.publish(qrcode_number)


                #             qrcodes_detected.append(qrcode.data)
                #             rospy.logwarn("BLOCO PARA BASE %s" % qrcode.data)

                #             except KeyError:
                #                 rospy.logfatal("Dict_letter data error %s" % qrcode.data)

                #         first_detection = False


                #     dx, dy = getQRcodeDistances(img, qrcode)

                #     array_distance.data = [dx , dy]
                #     pub_qrcode_distance.publish(array_distance)


                # Verifica se o QRCODE ja foi detectado e transportado
                # if first_detection == True and qrcode.data != "":
                #     if qrcode.data in qrcodes_detected:
                #         rospy.logwarn("QRCODE %s JA DETECTADO" % qrcode.data)
                #         continue
                #     else:
                #         time.sleep(2)
                #         pub_found_qrcode.publish(True)

                #         try:
                #             qrcode_number = DICT_LETTERS_NUMBERS[ qrcode.data ]

                #             pub.publish(String(data=qrcode.data))
                #             pub_number.publish(qrcode_number)

                #             qrcodes_detected.append(qrcode.data)
                #             rospy.logwarn("BLOCO PARA BASE %s" % qrcode.data)
                #         except KeyError:
                #             rospy.logfatal("Dict_letter data error %s" % qrcode.data)


                #     # print(qrcodes_detected)
                #     first_detection = False


                # dx, dy = getQRcodeDistances(img, qrcode)

                # array_distance.data = [dx , dy]
                # pub_qrcode_distance.publish(array_distance)

        cv2.imshow("QRCode_detection", img)
        cv2.waitKey(20)

    else:
        if start_detection == True:
            qrcodes_detected.append(qrcode_value)
            # print(qrcodes_detected)
            first_detection = True
            cv2.destroyAllWindows()
            start_detection = False


if __name__ == "__main__":
    rospy.init_node("gas_detector", anonymous=False)

    rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, image_callback, queue_size = 1)
    rospy.Subscriber("/block_detector/activate", Bool, activate_block_detection)

    rospy.spin()
