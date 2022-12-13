#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
# from mrs_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

ARENA_MAX = [8.5, 8.5, 4.0]
ARENA_MIN = [1.5, 1.5, 0.5]

CURRENT_POSITION = [8.25, 2.0, 0.71]

COASTAL_BASE = [8.25, 2.0, 0.71]
HANGING_BASE_1 = [5.05, 2.0, 2.5]
HANGING_BASE_2 = [8.0, 8.0, 2.5]
LANDING_BASE = [4.0, 4.0, 0.0]


sensor_in_range = Bool



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


def call_external_signal(true_or_false, last_time_sleep):
    if (true_or_false):
        rospy.loginfo("CALLING GREEN...")
        call_goto_relative(0.0, 0.0, 0.8, 0.0)
        time.sleep(5)

        call_goto_relative(0.0, 0.0, -0.8, 0.0)
        time.sleep(last_time_sleep)

    else:
        rospy.loginfo("CALLING RED...")
        call_goto_relative(0.0, 0.0, 0.0, -3.14)
        time.sleep(5)

        call_goto_relative(0.0, 0.0, 0.0, 3.14)
        time.sleep(last_time_sleep)



def goto_first_hanging_base():
    target_x = HANGING_BASE_1[0] - CURRENT_POSITION[0]
    target_y = HANGING_BASE_1[1] - CURRENT_POSITION[1]
    CURRENT_POSITION[0] = HANGING_BASE_1[0]
    CURRENT_POSITION[1] = HANGING_BASE_1[1]

    call_goto_relative(0.0, 0.0, 1.0, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(3.0)   

    call_goto_relative(target_x, target_y, 0.0, 0.0)
    rospy.loginfo("Going over the hanging base 1....")

    time.sleep(8.0)

    call_goto_relative(0.0, 0.0, -2.1, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(5.0)

def goto_second_hanging_base():
    target_x = HANGING_BASE_2[0] - CURRENT_POSITION[0]
    target_y = HANGING_BASE_2[1] - CURRENT_POSITION[1]
    CURRENT_POSITION[0] = HANGING_BASE_2[0]
    CURRENT_POSITION[1] = HANGING_BASE_2[1]

    call_goto_relative(0.0, 0.0, 1.0, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(3.0)   

    call_goto_relative(target_x, target_y, 0.0, 0.0)
    rospy.loginfo("Going over the hanging base 2....")

    time.sleep(10.0)

    call_goto_relative(0.0, 0.0, -1.95, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(5.0)

def goto_landing_base():
    target_x = LANDING_BASE[0] - CURRENT_POSITION[0]
    target_y = LANDING_BASE[1] - CURRENT_POSITION[1]
    CURRENT_POSITION[0] = LANDING_BASE[0]
    CURRENT_POSITION[1] = LANDING_BASE[1]

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(3.0)   

    call_goto_relative(target_x, target_y, 0.0, 0.0)
    rospy.loginfo("Going over the landing base....")

    time.sleep(10.0)

    call_goto_relative(0.0, 0.0, -1.6, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(5.0)

def goto_coastal_base():
    target_x = COASTAL_BASE[0] - CURRENT_POSITION[0]
    target_y = COASTAL_BASE[1] - CURRENT_POSITION[1]
    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    rospy.loginfo("Finding the right altitude...")

    time.sleep(3.0)   

    call_goto_relative(target_x, target_y, 0.0, 0.0)
    rospy.loginfo("Going over the coastal base....")

    time.sleep(12.0)

    call_goto_relative(0.0, 0.0, -2.7, 0.0)
    rospy.loginfo("Landing....")

    time.sleep(5)


def is_in_range(array):
    global sensor_in_range
    sensor_in_range = array.data[0]
    rospy.loginfo("IS IN RANGE? %r", sensor_in_range)


def mission_planner():
    global sensor_in_range

    time.sleep(10.0)

    activate_detection = rospy.Publisher("/gas_detector/activate", Bool, queue_size=10)

    # First arm
    call_arming(True)
    rospy.loginfo("Arming...")

    time.sleep(0.5)

    # Arm and took off
    call_set_mode("OFFBOARD", 4)
    rospy.loginfo("Mode Set")
    call_arming(True)
    rospy.loginfo("Mode set...tooking off...")
    time.sleep(10)



    # FIRST BASE
    goto_first_hanging_base()

    # INICIA O INSPECTION
    activate_detection.publish(True)

    array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray)

    activate_detection.publish(False)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO GAS
    rospy.loginfo("GAS NO RANGE CORRETO? %r", array.data[0])
    call_external_signal(array.data[0], 25)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO AJUSTE DE ZERO
    rospy.loginfo("AJUSTE DE ZERO NO RANGE CORRETO? %r", array.data[1])
    call_external_signal(array.data[1], 5)

    # AJUSTANDO POSICAO
    call_goto_relative(0.0, 0.0, 1.0, 0.0)    
    rospy.loginfo("Tooking off....")
    time.sleep(6.0)




    # SECONG BASE
    goto_second_hanging_base()

    # INICIA O INSPECTION
    activate_detection.publish(True)

    array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray)

    activate_detection.publish(False)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO GAS
    rospy.loginfo("GAS NO RANGE CORRETO? %r", array.data[0])
    call_external_signal(array.data[0], 25)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO AJUSTE DE ZERO
    rospy.loginfo("AJUSTE DE ZERO NO RANGE CORRETO? %r", array.data[1])
    call_external_signal(array.data[1], 5)

    # AJUSTANDO POSICAO
    call_goto_relative(0.0, 0.0, 1.0, 0.0)    
    rospy.loginfo("Tooking off....")
    time.sleep(6.0)




    # THIRD BASE
    goto_landing_base()

    # INICIA O INSPECTION
    activate_detection.publish(True)

    array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray)

    activate_detection.publish(False)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO GAS
    rospy.loginfo("GAS NO RANGE CORRETO? %r", array.data[0])
    call_external_signal(array.data[0], 25)

    # CHAMA O SINAL EXTERNO PARA O VALOR DO AJUSTE DE ZERO
    rospy.loginfo("AJUSTE DE ZERO NO RANGE CORRETO? %r", array.data[1])
    call_external_signal(array.data[1], 5)

    # AJUSTANDO POSICAO
    call_goto_relative(0.0, 0.0, 1.5, 0.0)    
    rospy.loginfo("Tooking off....")
    time.sleep(6.0)




    # BACK HOME
    goto_coastal_base()
    

if __name__ == "__main__": 
    rospy.init_node("inspection_task_main", anonymous=False)
    
    rospy.Subscriber("/gas_detector/in_range", ByteMultiArray, is_in_range)

    # r = rospy.Rate(5) # 10hz
    # while not rospy.is_shutdown():
    mission_planner() 
        # r.sleep()
        # break
    rospy.spin()