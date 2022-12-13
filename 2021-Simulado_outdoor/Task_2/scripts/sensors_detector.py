#! /usr/bin/env python3
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import sensors_class as sc
import cv2
import time, os, math
import numpy as np
from cv_bridge import CvBridge

mark = sc.Sensors()
bridge_ = CvBridge()

flag = False
flag_adjustment = False
pub_bad_sensor = rospy.Publisher('/sensor/detected_bad', Bool, queue_size=1)
pub_sensor_positions = rospy.Publisher('sensor/positions', Float32MultiArray, queue_size=1)

sensor_positions = Float32MultiArray()
sensor_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

START_POSITION = [7.75, 2.0]

total_detected = 0
flag_detection = False
detections_to_flag = 3
counter_false_detection = 0
current_sensor = 0
first_exec = True


def activate_sensor_detection(data):
    global flag
    global first_exec
    if data.data == True:
        # print("Detection Enabled!")
        first_exec = True
        flag = True
    else:
        # print("Detection Disabled!")
        flag = False

def image_callback(data):
    global flag
    global total_detected
    global detections_to_flag
    global counter_false_detection
    global flag_detection
    global first_exec
    global current_sensor
    global mark

    max_detections = 30

    if flag == True:
        if total_detected < max_detections:
            cvim = bridge_.imgmsg_to_cv2(data, "bgr8")
            cvim = cvim[:,-400:-345]
            # print(cvim.shape)
            mark.setImage(cvim, 0.)
            # print(type(data))

            # get reference frames
            R1, T1, success1, R2, T2, success2 = mark.getRefFrameSMarks()

            # if (success1):
                # rospy.loginfo("{}, {}, {}, {}".format(success1, total_detected, max_detections, flag_detection))
                # rospy.loginfo("{}".format(T1))

            if (success1 == True and total_detected < max_detections and flag_detection == True):
                # print("BAD Sensor Detected!")
                if (current_sensor < 6):
                    # pose = rospy.wait_for_message("/uav1/mavros/local_position/pose", PoseStamped, timeout=4.0)
                    # sensor_positions.data[current_sensor] = START_POSITION[0] + pose.pose.position.x
                    # sensor_positions.data[current_sensor + 1] = START_POSITION[1] + pose.pose.position.y

                    current_sensor += 1

                    # sensor_positions.data[6] += 1

                    # print(sensor_positions.data)

                    # rospy.loginfo("{}".format(sensor_positions.data))

                    pub_bad_sensor.publish(True)

                total_detected += 1

            if success1 == False:
                counter_false_detection += 1
            else:
                counter_false_detection = 0

            if (counter_false_detection >= detections_to_flag):
                flag_detection = True
            else:
                flag_detection = False

            # show
            mark.show()
            cv2.waitKey(30)

    else:
        cv2.destroyAllWindows()
        
        if first_exec == True:
            counter_false_detection = 0
            pub_sensor_positions.publish(sensor_positions)
            time.sleep(0.5)
            pub_sensor_positions.publish(sensor_positions)
            first_exec = False


if __name__ == "__main__":
    rospy.init_node("sensors_detector", anonymous=False)

    image = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, image_callback, queue_size = 1)
    rospy.Subscriber("/sensor_detector/activate", Bool, activate_sensor_detection)

    # image = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, activate_corner_adjustment, queue_size = 1)
    # rospy.Subscriber("/sensor_detector/adjustment", Bool, activate_adjustment)

    rospy.spin()
