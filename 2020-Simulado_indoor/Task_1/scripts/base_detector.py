#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
import landing_mark_class as lm
import cv2
import numpy as np
from cv_bridge import CvBridge

mark = lm.LandingMark()
bridge_ = CvBridge()
pub = rospy.Publisher('/base_detector/px_py', Float64MultiArray, queue_size=10)

def image_callback(data):
    cvim = bridge_.imgmsg_to_cv2(data, "bgr8")
    # print(cvim.shape)
    mark.setImage(cvim, 0.)
    # print(type(data))
    
    # get reference frame
    R, T, success = mark.getRefFrame()
	
    #print success
    (x, y, z) = T
    #(roll, pitch, yaw) = np.rad2deg(R)

    if success:
        msg = Float64MultiArray()
        msg.data = [x,y]
        pub.publish(msg)

        # print(x,y,z)
	
    # show
    mark.show()
    cv2.waitKey(30)


if __name__ == "__main__": 
    rospy.init_node("base_detector", anonymous=False)    

    image = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, image_callback, queue_size = 1)

    rospy.spin()
