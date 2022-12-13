#! /usr/bin/env python3
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
from std_srvs.srv import *
from mrs_msgs.msg import Float64Stamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
import time

COASTAL_BASE = [10.0, 90.0]
BAD_SENSORS_POSITIONS = []

class Uav:
    def __init__(self):
        self.relativePosition = Point(0.0, 0.0, 0.0)
        self.globalPosition = Point(10.0, 90.0, 19.0)
        self.globalStartPosition = Point(10.0, 90.0, 19.0)

    def getRelativePosition(self):
        pose = rospy.wait_for_message("/uav1/mavros/local_position/pose", PoseStamped, timeout=4.0)
        self.relativePosition = pose.pose.position
        return self.relativePosition

    def setRelativePosition(self, position: Point):
        self.relativePosition = position
        self.updateGlobalPosition()

    def getGlobalPosition(self):
        self.getRelativePosition()
        return self.globalPosition

    def updateGlobalPosition(self):
        self.getRelativePosition()
        self.globalPosition.x = self.globalStartPosition.x + self.relativePosition.x
        self.globalPosition.y = self.globalStartPosition.y + self.relativePosition.y
        self.globalPosition.z = self.globalStartPosition.z + self.relativePosition.z

    # Target deve ser uma posição global do gazebo
    def moveTo(self, targetX, targetY, thresholdX = 0, thresholdY = 0):
        self.updateGlobalPosition()

        distanceToTargetX = targetX - self.globalPosition.x
        distanceToTargetY = targetY - self.globalPosition.y

        # Distancia usada para calcular se o drone nao se moveu entre um movimento e outro
        # Funcão call_goto_relative nem sempre chega no ponto exato
        lastDistanceX = distanceToTargetX
        lastDistanceY = distanceToTargetY

        call_goto_relative(distanceToTargetX, distanceToTargetY, 0.0, 0.0)

        # Loop que verifica se o uav chegou no alvo. Enquanto não chegar não vai para o proximo hotspot
        while (abs(distanceToTargetX) > (5 - thresholdX) or abs(distanceToTargetY) > (5 - thresholdY)):
            time.sleep(5)

            self.updateGlobalPosition()

            distanceToTargetX = targetX - self.globalPosition.x
            distanceToTargetY = targetY - self.globalPosition.y

            # rospy.logwarn("Posicao global {:.2f} {:.2f}".format(self.globalPosition.x, self.globalPosition.y))
            # rospy.logwarn("Posicao relativa {:.2f} {:.2f}".format(self.relativePosition.x, self.relativePosition.y))
            # rospy.logwarn("Alvo {:.2f} {:.2f}".format(targetX, targetY))
            rospy.logwarn("Distancia até alvo {:.2f} {:.2f}\n".format(distanceToTargetX, distanceToTargetY))

            # Verifica se o drone está parado no mundo
            if (abs(lastDistanceX - distanceToTargetX) < 0.5 and abs(lastDistanceY - distanceToTargetY) < 0.5):
                rospy.logwarn("UAV parado. Movendo novamente...\n")
                call_goto_relative(distanceToTargetX, distanceToTargetY, 0.0, 0.0)
                time.sleep(4)

            lastDistanceX = distanceToTargetX
            lastDistanceY = distanceToTargetY

        time.sleep(1)
        rospy.logerr("Chegou no target")
    
    def moveAxisZ(self, relativeZ):
        global currentHeight

        self.updateGlobalPosition()

        targetZ = relativeZ + self.globalPosition.z

        call_goto_relative(0, 0, relativeZ, 0)

        time.sleep(2)

        while abs(relativeZ) > 0.35:
            time.sleep(0.5)

            self.updateGlobalPosition()

            # rospy.logwarn("Posicao global {:.2f} {:.2f} {:.2f}".format(self.globalPosition.x, self.globalPosition.y, self.globalPosition.z))
            # rospy.logwarn("Posicao relativa {:.2f} {:.2f} {:.2f}".format(self.relativePosition.x, self.relativePosition.y, self.relativePosition.z))
            # rospy.logwarn("Alvo Z: {:.2f}".format(targetZ))
            # rospy.logwarn("Distancia até alvo {:.2f}".format(relativeZ))
            # rospy.logwarn("Altura {:.2f}\n".format(currentHeight))

            relativeZ = targetZ - self.globalPosition.z

            call_goto_relative(0, 0, relativeZ, 0)

    def land(self):
        rospy.loginfo("Landing...")

        global currentHeight
        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        if (currentHeight >= 1.5):
            heightToGoDown = currentHeight - 0.5
            self.moveAxisZ(-heightToGoDown)
            rospy.logerr("Waiting heightToGoDown...")
            time.sleep(heightToGoDown - (heightToGoDown * 0.6))

        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        call_goto_relative(0.0, 0.0, -currentHeight - 0.5, 0.0)
        time.sleep(0.8)

        rospy.loginfo("Land finished...")

    def hover(self):
        global currentHeight
        
        while(currentHeight > 1.2 or currentHeight < 0.8):
            rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
            heightToMove = 1.0 - currentHeight 
            self.moveAxisZ(heightToMove)
            time.sleep(0.8)


    def landFinal(self):
        print("\n" + COLORS.BOLD + COLORS.CYAN + "Landing..." + COLORS.END + COLORS.END + "\n" )

        global currentHeight
        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        if (currentHeight >= 1):
            heightToGoDown = currentHeight - 0.4
            self.moveAxisZ(-heightToGoDown)
            time.sleep(heightToGoDown - (heightToGoDown * 0.6))

        call_uav_manager_land()
        print("\n" + COLORS.BOLD + COLORS.GREEN + "Land finished..." + COLORS.END + COLORS.END + "\n" )

uav = Uav()

currentHeight = 0
detected_bad = Bool

COASTAL_BASE = Point(10.0, 90.0, 0.71)
START_TUBE_EDGE = Point(-50, -25, 10)
FINAL_TUBE_EDGE = Point(-50, -44.5, 10)

class COLORS:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

activate_sensor = rospy.Publisher("/sensor_detector/activate", Bool, queue_size=10)
activate_adjustment = rospy.Publisher("/sensor_detector/adjustment", Bool, queue_size=10)

def call_uav_manager_land():
    try:
        service = rospy.ServiceProxy("/uav1/uav_manager/land", Trigger)
        rospy.wait_for_service("/uav1/uav_manager/land")

        print(service())
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)


def call_goto_relative(x, y, z, rxy):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/goto_relative", Vec4)
        rospy.wait_for_service("/uav1/control_manager/goto_relative")

        service([x,y,z,rxy])

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
        rospy.wait_for_service("/uav1/mavros/set_mode")

        print(service(mode_ID, mode))

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_arming(value):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav1/mavros/cmd/arming")

        print(service(value))

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_safety_area(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/use_safety_area", SetBool)
        rospy.wait_for_service("/uav1/control_manager/use_safety_area")

        print(service(value))

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_min_height(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/set_min_height", Float64Srv)
        rospy.wait_for_service("/uav1/control_manager/set_min_height")

        print(service(value))

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_external_signal():
    global detected_bad

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    time.sleep(5)

    call_goto_relative(0.0, 0.0, -0.5, 0.0)
    time.sleep(11)

    detected_bad = False


def callback_detected_bad(data):
    global detected_bad
    detected_bad = data.data


def callback_height(data):
    global currentHeight
    currentHeight = data.value


def land():
    global currentHeight
    rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
    if (currentHeight >= 1.5):
        heightToGoDown = currentHeight - 0.5
        call_goto_relative(0.0, 0.0, -heightToGoDown, 0.0)
        time.sleep(round(heightToGoDown, 1))

    rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
    call_goto_relative(0.0, 0.0, -currentHeight, 0.0)
    time.sleep(0.8)


def goto_first_edge():
    uav.moveTo(START_TUBE_EDGE.x, START_TUBE_EDGE.y, thresholdX=4.5, thresholdY=4.5)


def goto_final_edge():
    uav.moveTo(FINAL_TUBE_EDGE.x, FINAL_TUBE_EDGE.y, thresholdX=4.5, thresholdY=4.5)


def go_through_pipe_stop_on_bad(steps = 25):
    global detected_bad, currentHeight

    numberSensorsDetected = 0

    uav.updateGlobalPosition()

    deltaX = (FINAL_TUBE_EDGE.x - uav.globalPosition.x) / steps
    deltaY = (FINAL_TUBE_EDGE.y - uav.globalPosition.y) / steps

    # Faz a verificacao ao iniciar a imagem do cano
    if (detected_bad == True):
        numberSensorsDetected += 1
        activate_sensor.publish(False)

        uav.updateGlobalPosition()

        print("\n" + COLORS.BOLD + COLORS.GREEN + "Sensor ruim detectado na posicao [{:.2f}, {:.2f}]. Altura {:.2f}m".format(uav.globalPosition.x, uav.globalPosition.y + 0.3, uav.relativePosition.z - 0.15) + COLORS.END + COLORS.END + "\n" )
        detected_bad = False
        time.sleep(4)

        call_goto_relative(deltaX, deltaY, 0.0, 0.0)
        activate_sensor.publish(True)
        time.sleep(0.35)

    rospy.loginfo("Moving...")

    diff_x = abs(uav.globalPosition.x - FINAL_TUBE_EDGE.x)
    diff_y = abs(uav.globalPosition.y - FINAL_TUBE_EDGE.y)

    while ( diff_x > 0.1 or diff_y > 0.15 ):
        uav.updateGlobalPosition()

        if (detected_bad == True):
            numberSensorsDetected += 1
            activate_sensor.publish(False)
            
            print("\n" + COLORS.BOLD + COLORS.GREEN + "Sensor ruim detectado na posicao [{:.2f}, {:.2f}]. Altura {:.2f}m".format(uav.globalPosition.x, uav.globalPosition.y + 0.3, uav.relativePosition.z - 0.15) + COLORS.END + COLORS.END + "\n" )
            detected_bad = False
            time.sleep(3.5)

            # call_external_signal()

            if (numberSensorsDetected == 3):
                print("\n" + COLORS.BOLD + COLORS.CYAN + "Todos sensores vermelhos detectados!" + COLORS.END + COLORS.END + "\n")
                break

            call_goto_relative(deltaX, deltaY, 0.0, 0.0)
            activate_sensor.publish(True)
            time.sleep(0.1)
        else:
            time.sleep(0.4)
            call_goto_relative(deltaX, deltaY, 0.0, 0.0)
            time.sleep(0.1)

        diff_x = abs(uav.globalPosition.x - FINAL_TUBE_EDGE.x)
        diff_y = abs(uav.globalPosition.y - FINAL_TUBE_EDGE.y)


def go_through_pipe(steps):
    global detected_bad

    uav.updateGlobalPosition()

    deltaX = (FINAL_TUBE_EDGE.x - uav.globalPosition.x) / steps
    deltaY = (FINAL_TUBE_EDGE.y - uav.globalPosition.y) / steps

    rospy.logwarn("Distance per step: [ %s, %s ]" % (deltaX, deltaY))

    # Faz a verificacao ao iniciar a imagem do cano
    if (detected_bad == True):
        rospy.logwarn("Sensor ruim detectado! Armazenando... Total de sensores ruins detectados: {}".format(len(BAD_SENSORS_POSITIONS)))

        uav.updateGlobalPosition()
        BAD_SENSORS_POSITIONS.append(uav.globalPosition)

        detected_bad = False
        # call_external_signal()

    for i in range(0, steps):

        rospy.loginfo("Moving...")

        call_goto_relative(deltaX, deltaY, 0.0, 0.0)

        time.sleep(5)

        if (detected_bad == True):
            rospy.logwarn("Sensor ruim detectado! Armazenando... Total de sensores ruins detectados: {}".format(len(BAD_SENSORS_POSITIONS)))

            uav.updateGlobalPosition()
            BAD_SENSORS_POSITIONS.append(uav.globalPosition)

            detected_bad = False
            # call_external_signal()


def goto_sensors():
    rospy.loginfo("Going to sensors...")
    count = 0
    for badSensorPosition in BAD_SENSORS_POSITIONS:
        count += 1

        uav.moveTo(badSensorPosition.x, badSensorPosition.y, thresholdX=4.6, thresholdY=4.6)

        time.sleep(8.5)

        print("\n" + COLORS.BOLD + COLORS.GREEN + "Sensor encontrado na posicao ({}, {})".format(round(badSensorPosition.x, 2), round(badSensorPosition.y, 2)) + COLORS.END + COLORS.END)

        call_external_signal()

    print("\n" + COLORS.BOLD + COLORS.CYAN + "Sensores detectados percorridos..." + COLORS.END + COLORS.END + "\n")


def goto_coastal_base():
    rospy.logwarn("Going over the coastal base...\n")

    uav.moveTo(COASTAL_BASE.x, COASTAL_BASE.y, thresholdX = 4.5, thresholdY = 4.5)


def mission_planner():
    uav.updateGlobalPosition()

    rospy.logwarn("Iniciando...\n")
    rospy.logwarn("Posicao relativa:\t{:.2f} {:.2f}".format(uav.relativePosition.x, uav.relativePosition.y))
    rospy.logwarn("Posicao global:\t{:.2f} {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y))

    rospy.loginfo("Finding the right altitude...\n")
    uav.moveAxisZ(4)
    time.sleep(10)

    goto_first_edge()

    uav.hover()
    time.sleep(1.5)

    activate_sensor.publish(True)

    STEPS = 28
    go_through_pipe_stop_on_bad(STEPS)

    activate_sensor.publish(False)

    uav.moveAxisZ(4.5)
    time.sleep(5)

    # Evita colisão com a parede
    goto_first_edge()

    goto_coastal_base()

    uav.landFinal()



    # First arm
    # call_arming(True)
    # rospy.loginfo("Arming...")

    # time.sleep(0.5)

    # Arm and took off
    # call_set_mode("OFFBOARD", 4)
    # rospy.loginfo("Mode Set")

    # Second arm
    # call_arming(True)
    # rospy.loginfo("Mode set...tooking off...")

    # time.sleep(14)

    # call_goto_relative(0.0, 0.0, -0.8, 0.0)
    # rospy.loginfo("Findind the right altitute...")

    # time.sleep(6.0)


    # activate_sensor.publish(True)


    # STEPS = 25
    # go_through_pipe_stop_on_bad(STEPS)


    # activate_sensor.publish(False)


    # positions = rospy.wait_for_message('sensor/positions', Float32MultiArray)

    # SENSORS = [[positions.data[0], positions.data[1]], [positions.data[2], positions.data[3]], [positions.data[4], positions.data[5]]]
    # qtd_sensors_detected = positions.data[6]


    # goto_sensors(SENSORS, qtd_sensors_detected)


    # call_goto_relative(0.0, 0.0, -1.0, 0.0)
    # rospy.loginfo("Findind the right altitute...")

    # time.sleep(5.0)

    # call_goto_relative(-2.75, 1.0, 0.0, 0.0)
    # rospy.loginfo("Going over the pipe's edge....")

    # time.sleep(15.0)

    # activate_sensor.publish(True)
    # rospy.loginfo("Going through the pipe....")

    # call_goto_relative(-1.0, 5.0, 0.0, 0.0)go_through_pipe
    # call_goto_relative(0.0, 0.0, 1.0, 0.0)
    # rospy.loginfo("Going up...")

    # time.sleep(5)

    # call_goto_relative(3.75, -6.0, 0.0, 0.0)
    # rospy.loginfo("Going back home....")

    # time.sleep(20)

    # call_goto_relative(0.0, 0.0, -3.0, 0.0)
    # rospy.loginfo("Landing....")

if __name__ == "__main__":

    time.sleep(12.0)

    rospy.init_node("sensing_task_node", anonymous=False)
    call_safety_area(False)
    call_min_height(0.0)

    rospy.Subscriber("/sensor/detected_bad", Bool, callback_detected_bad)
    rospy.Subscriber("/uav1/odometry/height", Float64Stamped, callback_height)
    # rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, callback_pose)

    mission_planner()

    rospy.spin()
