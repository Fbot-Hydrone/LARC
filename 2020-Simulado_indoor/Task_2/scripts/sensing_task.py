#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
from std_srvs.srv import *
# from mrs_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import time

# all_detected = Bool

def call_goto_relative(x, y, z, rxy):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/goto_relative", Vec4)
        rospy.wait_for_service("/uav1/control_manager/goto_relative")

        print(service([x,y,z,rxy]))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e  

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
        rospy.wait_for_service("/uav1/mavros/set_mode")

        print(service(mode_ID, mode))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e  

def call_arming(value):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav1/mavros/cmd/arming")

        print(service(value))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def call_safety_area(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/use_safety_area", SetBool)
        rospy.wait_for_service("/uav1/control_manager/use_safety_area")

        print(service(value))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def call_min_height(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/set_min_height", Float64Srv)
        rospy.wait_for_service("/uav1/control_manager/set_min_height")

        print(service(value))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

# def all_detected_callback(data):
#     global all_detected
#     all_detected = data

def mission_planner():

    activate_sensor = rospy.Publisher("/sensor_detector/activate", Bool, queue_size=10)

    # First arm
    call_arming(True)
    rospy.loginfo("Arming...")

    time.sleep(0.5)

    # Arm and took off
    call_set_mode("OFFBOARD", 4)
    rospy.loginfo("Mode Set")
    call_arming(True)
    rospy.loginfo("Mode set...tooking off...")

    time.sleep(15)

    call_goto_relative(0.0, 0.0, -1.0, 0.0)
    rospy.loginfo("Findind the right altitute...")

    time.sleep(5.0)

    call_goto_relative(-2.75, 1.0, 0.0, 0.0)
    rospy.loginfo("Going over the can's edge....")

    time.sleep(15.0)

    activate_sensor.publish(True)
    rospy.loginfo("Going through the can....")
   
    call_goto_relative(-1.0, 5.0, 0.0, 0.0)

    time.sleep(20)

    activate_sensor.publish(False)

    call_goto_relative(0.0, 0.0, 1.0, 0.0)
    rospy.loginfo("Going up...")

    time.sleep(5)

    call_goto_relative(3.75, -6.0, 0.0, 0.0)
    rospy.loginfo("Going back home....")

    time.sleep(20)

    call_goto_relative(0.0, 0.0, -3.0, 0.0)
    rospy.loginfo("Landing....")    

if __name__ == "__main__": 

    time.sleep(10.0)
    
    rospy.init_node("sensing_task_node", anonymous=False)
    call_safety_area(False)
    call_min_height(0.0)

    # rospy.Subscriber("/sensor_detection_finished", Bool, all_detected_callback)
    
    mission_planner() 
    
    rospy.spin()
