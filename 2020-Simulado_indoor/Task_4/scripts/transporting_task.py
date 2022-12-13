#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
# from mrs_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import time
from geometry_msgs.msg import *
from gazebo_ros_link_attacher.srv import *

position = PoseStamped()

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

def call_attacher(id):
    try:
        service = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        rospy.wait_for_service("/link_attacher_node/attach")

        print(service('uav1', 'base_link', 'equipment'+id, 'link_'+id))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e
        
def call_detacher(id):
    try:
        service = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        rospy.wait_for_service("/link_attacher_node/detach")

        print(service('uav1', 'base_link', 'equipment'+id, 'link_'+id))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def pose_callback(data):
    global position
    position = data

def goto_first_hanging_base():
    global position

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(5.0)   

    call_goto_relative(-3.1, 0., 0.0, 0.0)
    rospy.loginfo("Going over the hanging base 1....")

    time.sleep(15.0)

    call_goto_relative(0.0, 0.0, -1.75, 0.0)
    rospy.loginfo("Going down....")

    time.sleep(15.0)

    call_attacher('D')

    time.sleep(1.0)

    call_goto_relative(0.0, 0.0, 1.75, 0.0)
    rospy.loginfo("Going up....")

    time.sleep(10.0)

def goto_second_hanging_base():
    global position

    call_goto_relative(2.75, 6.1, 0.0, 0.0)
    rospy.loginfo("Going over the hanging base 2....")

    time.sleep(20.0)

    call_goto_relative(0.0, 0.0, -1.75, 0.0)
    rospy.loginfo("Going down....")

    time.sleep(15.0)

    call_detacher('D')

    time.sleep(1.0)

    call_goto_relative(0.0, 0.0, 1.75, 0.0)
    rospy.loginfo("Going up....")

    time.sleep(10.0)  

def mission_planner():
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

    goto_first_hanging_base()    

    goto_second_hanging_base()    

if __name__ == "__main__": 
    rospy.init_node("transporting_task_node", anonymous=False)  

    rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, pose_callback, queue_size = 10)

    mission_planner() 

    rospy.spin()
