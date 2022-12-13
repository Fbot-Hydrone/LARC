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

local_pos_x = 0.0
local_pos_y = 0.0

hotspots = [[0.0, 3.0], [0.0, 4.5], [-2.0, 5.0], [-4.0, 5.0], [-6.0, 5.0], \
            [-6.0, 3.0], [-5.5, 0.0], [-4.0, 3.0], [-2.0, 3.0], [-1.5, 1.0]]

finding_base_altitute = 1.0

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

def pose_callback(data):
    global position
    position = data

def find_base():
    rospy.loginfo("Finding a Base.....")

    delta_x = 0
    delta_y = 0

    success = False
    PX_PY_THRESHOLD = 50
    ADJUSTED_STEP = 0.1
 
    while True:
        try:
            value = rospy.wait_for_message("/base_detector/px_py", Float64MultiArray, 1.0)
        except(rospy.ROSException), e:
            print e   
            rospy.loginfo("NO BASE FOUND!")  
            break               

        if value.data[0] > PX_PY_THRESHOLD:
            delta_y = -ADJUSTED_STEP
        elif value.data[0] < -PX_PY_THRESHOLD:
            delta_y = ADJUSTED_STEP
        
        if value.data[1] > PX_PY_THRESHOLD:
            delta_x = -ADJUSTED_STEP
        elif value.data[1] < -PX_PY_THRESHOLD:
            delta_x = ADJUSTED_STEP

        if ((abs(value.data[0]) < PX_PY_THRESHOLD) and (abs(value.data[1]) < PX_PY_THRESHOLD)):
            rospy.loginfo("Over the detected base")  
            # print(value.data[0], value.data[1])
            success = True
            break
        
        rospy.loginfo("APPROACHING BASE....") 

        call_goto_relative(delta_x, delta_y, 0.0, 0.0)
        time.sleep(0.1)

    return success 

def goto_first_hanging_base():
    global local_pos_x
    global local_pos_y 

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(5.0)   

    call_goto_relative(-3.0, 0.0, 0.0, 0.0)
    local_pos_x += -3.0
    rospy.loginfo("Going over the hanging base 1....")

    time.sleep(15.0)

    call_goto_relative(0.0, 0.0, -2.5, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(8.0)

    call_goto_relative(0.0, 0.0, 1.5, 0.0)    
    rospy.loginfo("Tooking off....")

    time.sleep(10.0)

def goto_second_hanging_base():
    global local_pos_x
    global local_pos_y

    call_goto_relative(2.75, 6.1, 0.0, 0.0)
    local_pos_y += 6.1
    local_pos_x += 2.75
    rospy.loginfo("Going over the hanging base 2....")

    time.sleep(20.0)

    call_goto_relative(0.0, 0.0, -2.5, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(8.0)    

def goto_hotspots():
    global local_pos_x
    global local_pos_y
    global hotspots
    global position
    global finding_base_altitute

    landed_bases_pos = []
    
    for i in range(0, len(hotspots)):
        call_goto_relative(hotspots[i][0] - local_pos_x, hotspots[i][1] - local_pos_y, 0.0, 0.0)
        local_pos_x += hotspots[i][0] - local_pos_x
        local_pos_y += hotspots[i][1] - local_pos_y

        rospy.loginfo("Going over the %s hotspot....", i)

        time.sleep(10.0)

        flag_found_base = find_base()

        if (flag_found_base): 
            flag_already_landed = False

            local_pos_x = position.pose.position.x
            local_pos_y = position.pose.position.y

            for i in range(0, len(landed_bases_pos)):                

                if ((abs(local_pos_x - landed_bases_pos[i][0]) < 1.0) and (abs(local_pos_y - landed_bases_pos[i][1]) < 1.0)):
                    flag_already_landed = True
            
            if (not flag_already_landed):
                rospy.loginfo("Landing over detected base....")
                rospy.loginfo("Detected base position: (%s, %s) ....", local_pos_x, local_pos_y)

                call_goto_relative(0.0, 0.0, -2.5, 0.0)
                rospy.loginfo("Landing....")

                landed_bases_pos.append([local_pos_x, local_pos_y])

                time.sleep(8.0)

                call_goto_relative(0.0, 0.0, finding_base_altitute, 0.0)
                rospy.loginfo("Tooking off....")

                time.sleep(5.0)

        if (len(landed_bases_pos) == 3):
            break

        rospy.loginfo("FCU: (%s, %s) ....", hotspots[i][0] - local_pos_x, hotspots[i][1] - local_pos_y)
        rospy.loginfo("Actual position: (%s, %s) ....", local_pos_x, local_pos_y)   

    call_goto_relative(-3.0 - local_pos_x, 3.0 - local_pos_y, 0.0, 0.0) 
    rospy.loginfo("Going to arena's center.....")

    time.sleep(10.0) 

    local_pos_x = position.pose.position.x
    local_pos_y = position.pose.position.y  

    call_goto_relative(0.0 - local_pos_x, 0.0 - local_pos_y, 0.0, 0.0) 

    time.sleep(10.0)

    rospy.loginfo("Going back home.....")

    call_goto_relative(0.0, 0.0, -2.5, 0.0)
    rospy.loginfo("Landing....")

    # call_goto_relative(0.0, 2.0, 0.0, 0.0)
    # local_pos_y = 2.0
    # rospy.loginfo("Going over second hotspot....")

    # time.sleep(10.0)

def mission_planner():
    global finding_base_altitute

    time.sleep(10.0)

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

    call_goto_relative(0.0, 0.0, finding_base_altitute, 0.0)
    rospy.loginfo("Tooking off....")
    
    # call_goto_relative(0.0, 0.0, -0.5, 0.0)
    # rospy.loginfo("Tooking off....")

    time.sleep(5.0)

    goto_hotspots()    

if __name__ == "__main__": 
    rospy.init_node("mapping_task_node", anonymous=False)  

    rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, pose_callback, queue_size = 10)

    mission_planner() 

    rospy.spin()
