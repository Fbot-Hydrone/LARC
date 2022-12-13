#! /usr/bin/env python3
import time
import rospy
from rospy.core import xmlrpcapi
from std_msgs.msg import *
from mrs_msgs.srv import *
from std_srvs.srv import *
from sensor_msgs.msg import *
from mrs_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion

from threading import Thread
import numpy as np
import sympy as sp
from sympy.matrices import Matrix
import math

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


currentHeight = 0
findingBaseAltitude = 5.0

# Coordenadas do gazebo
START_TUBE_EDGE = Point(-50, -25, 10)
FINAL_TUBE_EDGE = Point(-50, -44.5, 10)
COASTAL_BASE = Point(10, 90, 0)
FIXED_BASE_1 = Point(45, 10, 2.6)
FIXED_BASE_2 = Point(-19.5, -21, 13.6)
FIXED_BASE_3 = Point(-54, -35, 9.85)
BASE_SIZE = 1 # m
distanciaFocal = 215.68

# Hotspot de teste (faz um simbolo de '+' no mapa)
""" WAYPOINTS = [
    [0, 0],
    [-50, 0],
    [0, 0],
    [50, 0],
    [0, 0],
    [0, -50],
    [0, 0],
    [0, 50],
] """

WAYPOINTS = [
    [22, 25],
    [24, 10],
    [24, -10],
    [24, -35],
    [0, -22],
    [0, 4],
    [0, 25],
    [-18, 25],
    [-25, 3],
    [-41, 3],
    [-40, 32],
    [-58, 33],
    [-60, 0],
    [-90, 40],
    [-110.52, 75],
    [-70, -15],
    [-115, -30],
    [-100, -70],
    [-109.5, -112.67],
    [-60, -90],
    [-10, -40],
    [-40, -100],
    [30, -70],
    [60, -100],
    [90, -60],
    [82, -15],
    [113, -60],
    [113, -100]
]

FB = False
VISITED_BASES = 0

# Hotspot para testar o pouso (vai na posicao perto da base e volta para ela)
# WAYPOINTS = [
#     [10, 80]
# ]

class Uav:
    def __init__(self) -> None:
        self.stopMovingFlag = False
        self.isMoving = False
        self.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.lastPosition = Point(0.0, 0.0, 0.0)
        self.relativePosition = Point(0.0, 0.0, 0.0)
        self.globalPosition = Point(10.0, 90.0, 10.0)
        self.globalStartPosition = Point(10.0, 90.0, 10.0)
        self.landedBases = [[COASTAL_BASE, 'fixa']] # Coastal base precisa ser incluida pois o drone pode detectar ela e tente pousar
        self.maxLandBases = 7 # 1 coastal + 3 fixed bases + 3 moving bases
        self.movingBasesLanded = 0 # 1 coastal + 3 fixed bases + 3 moving bases

    def getRelativePosition(self):
        pose = rospy.wait_for_message("/uav1/mavros/local_position/pose", PoseStamped, timeout=4.0)
        self.relativePosition = pose.pose.position
        self.orientation = pose.pose.orientation
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
        if (self.isMoving):
            return

        self.isMoving = True

        self.updateGlobalPosition()

        distanceToTargetX = targetX - self.globalPosition.x
        distanceToTargetY = targetY - self.globalPosition.y

        # Distancia usada para calcular se o drone nao se moveu entre um movimento e outro
        # Funcão call_goto_relative nem sempre chega no ponto exato
        lastDistanceX = distanceToTargetX
        lastDistanceY = distanceToTargetY

        self.lastPosition.x = self.globalPosition.x
        self.lastPosition.y = self.globalPosition.y
        self.lastPosition.z = self.globalPosition.z

        call_goto_relative(distanceToTargetX, distanceToTargetY, 0.0, 0.0)

        # Loop que verifica se o uav chegou no alvo. Enquanto não chegar não vai para o proximo hotspot
        while (abs(distanceToTargetX) > (5 - thresholdX) or abs(distanceToTargetY) > (5 - thresholdY)):
            time.sleep(10)

            if (self.stopMovingFlag):
                call_goto_relative(0, 0, 0, 0)
                time.sleep(2)
                self.stopMovingFlag = False
                self.isMoving = False
                return

            self.updateGlobalPosition()

            distanceToTargetX = targetX - self.globalPosition.x
            distanceToTargetY = targetY - self.globalPosition.y

            self.lastPosition.x = self.globalPosition.x
            self.lastPosition.y = self.globalPosition.y
            self.lastPosition.z = self.globalPosition.z

            # rospy.logwarn("Posicao global {:.2f} {:.2f}".format(self.globalPosition.x, self.globalPosition.y))
            # rospy.logwarn("Posicao relativa {:.2f} {:.2f}".format(self.relativePosition.x, self.relativePosition.y))
            # rospy.logwarn("Alvo {:.2f} {:.2f}".format(targetX, targetY))
            rospy.logwarn("Distancia até alvo {:.2f} {:.2f}\n".format(distanceToTargetX, distanceToTargetY))

            # Verifica se o drone está parado no mundo
            if (abs(lastDistanceX - distanceToTargetX) < 0.5 and abs(lastDistanceY - distanceToTargetY) < 0.5):
                rospy.logerr("UAV parado. Movendo novamente...\n")
                call_goto_relative(distanceToTargetX, distanceToTargetY, 0.0, 0.0)
                time.sleep(3.5)

            lastDistanceX = distanceToTargetX
            lastDistanceY = distanceToTargetY

        time.sleep(0.2)
        rospy.logerr("Chegou no target")

        self.isMoving = False

    def moveAxisZ(self, relativeZ):
        global currentHeight

        self.updateGlobalPosition()

        targetZ = relativeZ + self.globalPosition.z

        call_goto_relative(0, 0, relativeZ, 0)

        time.sleep(2)

        while abs(relativeZ) > 0.35:
            time.sleep(0.5)

            self.updateGlobalPosition()

            rospy.logwarn("Posicao global {:.2f} {:.2f} {:.2f}".format(self.globalPosition.x, self.globalPosition.y, self.globalPosition.z))
            rospy.logwarn("Posicao relativa {:.2f} {:.2f} {:.2f}".format(self.relativePosition.x, self.relativePosition.y, self.relativePosition.z))
            rospy.logwarn("Alvo Z: {:.2f}".format(targetZ))
            rospy.logwarn("Distancia até alvo {:.2f}".format(relativeZ))
            rospy.logwarn("Altura {:.2f}\n".format(currentHeight))

            relativeZ = targetZ - self.globalPosition.z

            call_goto_relative(0, 0, relativeZ, 0)

    def stopMoving(self):
        if (not self.isMoving):
            return

        self.stopMovingFlag = True

        rospy.logwarn("Stoping...")

        time.sleep(1)

    def land(self):
        rospy.loginfo("Landing...")

        global currentHeight
        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        if (currentHeight >= 1.5):
            heightToGoDown = currentHeight - 0.5
            uav.moveAxisZ(-heightToGoDown)
            time.sleep(heightToGoDown - (heightToGoDown * 0.6))

        while (currentHeight > 1):
            uav.moveAxisZ(-(currentHeight + 0.4))
            time.sleep(1)

        call_goto_relative(0, 0, -(currentHeight + 0.5), 0)
        time.sleep(2)

        rospy.loginfo("Land finished...")

    def landFinal(self):
        rospy.loginfo("Landing...")

        global currentHeight
        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        if (currentHeight >= 1.5):
            heightToGoDown = currentHeight - 0.5
            uav.moveAxisZ(-heightToGoDown)
            time.sleep(heightToGoDown - (heightToGoDown * 0.6))

        while (currentHeight > 1):
            uav.moveAxisZ(-currentHeight)
            time.sleep(1)

        call_uav_manager_land()

    def alreadyLandedBase(self, x = None, y = None):
        self.updateGlobalPosition()

        if (not x and not y):
            x = self.globalPosition.x
            y = self.globalPosition.y

        for base in self.landedBases:
            if (base[1] == 'fixa'):
                if abs(base[0].x - x) < 1 and abs(base[0].y - y) < 1:
                    return True

            if abs(base[0].x - x) < 7 and abs(base[0].y - y) < 7:
                return True

        return False

uav = Uav()

def goto_first_edge():
    uav.moveTo(START_TUBE_EDGE.x, START_TUBE_EDGE.y, thresholdX=4.7, thresholdY=4.7)

def goto_final_edge():
    uav.moveTo(FINAL_TUBE_EDGE.x, FINAL_TUBE_EDGE.y, thresholdX=4.7, thresholdY=4.7)

def get_tube_center():
    centerX = (START_TUBE_EDGE.x + FINAL_TUBE_EDGE.x) / 2
    centerY = (START_TUBE_EDGE.y + FINAL_TUBE_EDGE.y) / 2

    return Point(centerX, centerY, START_TUBE_EDGE.z)

def estimateBasePosition(dxPixel, dyPixel, row, pitch):
    relativeX = (dxPixel * (uav.globalPosition.z + 1.4)) / (math.cos(row) * distanciaFocal)
    relativeY = (dyPixel * (uav.globalPosition.z + 1.4)) / (math.cos(pitch) * distanciaFocal)

    uav.updateGlobalPosition()

    globalX = uav.globalPosition.x + relativeX
    globalY = uav.globalPosition.y + relativeY

    return globalX, globalY

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

def call_uav_manager_land():
    try:
        service = rospy.ServiceProxy("/uav1/uav_manager/land", Trigger)
        rospy.wait_for_service("/uav1/uav_manager/land")

        service()

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)


def callback_height(data):
    global currentHeight
    currentHeight = data.value


def findBase():
    global currentHeight

    print("\n" + COLORS.BOLD + COLORS.GREEN + "Finding base..." + COLORS.END + COLORS.END)

    delta_x = 0
    delta_y = 0
    delta_z = -0.3

    width = 50
    height = 50

    firstMove = True

    success = False
    PX_THRESHOLD = width/2
    PY_THRESHOLD = height/2
    ADJUSTED_STEP = 0.4

    biggestWidth = 0
    biggestHeight = 0

    while True:
        uav.updateGlobalPosition()

        quaternion = (
            uav.orientation.x,
            uav.orientation.y,
            uav.orientation.z,
            uav.orientation.w)

        euler = euler_from_quaternion(quaternion)

        if (abs(euler[0]) > 0.025 or abs(euler[1]) > 0.03):
            time.sleep(0.22)
            continue

        try:
            # Distancia em pixels retornada do algoritmo de detecção de base (base_detector.cpp)
            distancePixels = rospy.wait_for_message("/base_detector/px_py", Float64MultiArray, 3)
        except(rospy.ROSException) as e:
            print(COLORS.BOLD + COLORS.GREEN + "Base not found" + COLORS.END + COLORS.END + "\n" )
            break

        try:
            baseSize = rospy.wait_for_message("base_detector/base_size", Float64MultiArray, 2)
            width = baseSize.data[0]
            height = baseSize.data[1]

            if (biggestWidth < width):
                biggestWidth = width

            if (biggestHeight < height):
                biggestHeight = height

            if (width < 43):
                PX_THRESHOLD = width/2
            elif (width < 55):
                PX_THRESHOLD = width/3.5
            else:
                PX_THRESHOLD = width/4.5

            if (height < 43):
                PY_THRESHOLD = height/2
            elif (height < 55):
                PY_THRESHOLD = height/3.5
            else:
                PY_THRESHOLD = height/4.5

            print("\n" + COLORS.BOLD + COLORS.GREEN + "Threshold ({:.2f}, {:.2f})".format(PX_THRESHOLD, PY_THRESHOLD) + COLORS.END + COLORS.END)
            print(COLORS.BOLD + COLORS.CYAN + "Width (w: {:.1f}, h: {:.1f})".format(width, height) + COLORS.END + COLORS.END)

        except(rospy.ROSException) as e:
            rospy.loginfo("Using default baseSize")
            break

        print(COLORS.BOLD + COLORS.BLUE + "Distances (dx: {:.1f}, dy: {:.1f})".format(distancePixels.data[1], distancePixels.data[0]) + COLORS.END + COLORS.END)

        if firstMove:
            globalX, globalY = estimateBasePosition(distancePixels.data[1], -distancePixels.data[0], euler[1], euler[0])

            print(COLORS.BOLD + COLORS.BLUE + "Estimativa posicao (x: {:.1f}, y: {:.1f})".format(globalX, globalY) + COLORS.END + COLORS.END)

            if (uav.alreadyLandedBase(globalX, globalY)):
                print("\n" + COLORS.BOLD + COLORS.GREEN + "Target base already landed..." + COLORS.END + COLORS.END + "\n" )
                success = False
                break

            uav.moveTo(globalX, globalY, 4.8, 4.8)
            uav.moveAxisZ(7 - uav.globalPosition.z)
            firstMove = False
            time.sleep(1)
            continue

        if distancePixels.data[0] > PY_THRESHOLD:
            delta_y = -ADJUSTED_STEP
        elif distancePixels.data[0] < -PY_THRESHOLD:
            delta_y = ADJUSTED_STEP
        else:
            delta_y = 0.0

        if distancePixels.data[1] > PX_THRESHOLD:
            delta_x = ADJUSTED_STEP
        elif distancePixels.data[1] < -PX_THRESHOLD:
            delta_x = -ADJUSTED_STEP
        else:
            delta_x = 0.0

        if (width >= 50 and height >= 50):
            delta_z = 0

        print(COLORS.BOLD + COLORS.GREEN + "Moving: ({:.2f}, {:.2f}, {:.2f})".format(delta_x, delta_y, delta_z) + COLORS.END + COLORS.END + "\n" )

        call_goto_relative(delta_x, delta_y, delta_z, 0.0)
        time.sleep(1.7)

        if (abs(distancePixels.data[0]) < PY_THRESHOLD) and (abs(distancePixels.data[1]) < PX_THRESHOLD):
            rospy.loginfo("Over the detected base")

            heightToGoDown = currentHeight - (currentHeight * 0.55)

            if (currentHeight < 2):
                heightToGoDown = currentHeight + 0.4

            call_goto_relative(0, 0, -heightToGoDown, 0.0)
            time.sleep(2.5)

        if (currentHeight < 0.3):
            success = True
            break

        if (biggestHeight > 150 and biggestWidth > 150 and currentHeight < 1):
            if (uav.alreadyLandedBase()):
                print("\n" + COLORS.BOLD + COLORS.GREEN + "Target base already landed..." + COLORS.END + COLORS.END + "\n" )
                success = False
                break

            uav.land()
            success = True
            break

    return success

def findBase2():
    global COASTAL_BASE
    global FIXED_BASE_1
    global FIXED_BASE_2

    global FB
    global VISITED_BASES
    global currentHeight

    rospy.loginfo("Finding a Base...")

    while True:
        try:
            uav.updateGlobalPosition()

            if (abs(uav.orientation.x) > 0.012 or abs(uav.orientation.y) > 0.016):
                time.sleep(0.25)
                continue

            # Distancia em pixels retornada do algoritmo de detecção de base (base_detector.cpp)
            Pixels = rospy.wait_for_message("/base_detector/px_py", Float64MultiArray, 1.0)

            prelativeX = ((2*currentHeight*math.cos(math.radians(60)) * Pixels.data[0]) / Pixels.data[2]) + uav.relativePosition.x
            prelativeY = ((2*currentHeight*math.cos(math.radians(65)) * Pixels.data[1]) / Pixels.data[3]) + uav.relativePosition.y

            pglobalX = 10 + prelativeX
            pglobalY = 90 + prelativeY

            difcbx = abs(pglobalX - COASTAL_BASE.x) > 3
            difcby = abs(pglobalY - COASTAL_BASE.y) > 3
            diffb1x = abs(pglobalX - FIXED_BASE_1.x) > 3
            diffb1y = abs(pglobalY - FIXED_BASE_1.y) > 3
            diffb2x = abs(pglobalX - FIXED_BASE_2.x) > 3
            diffb2y = abs(pglobalY - FIXED_BASE_2.y) > 3

            if difcbx and difcby and diffb1x and diffb1y and diffb2x and diffb2y:
                rospy.loginfo("Found Base: {:.2f} {:.2f}\n".format(pglobalX, pglobalY))
                FB = True
                time.sleep(2)

                uav.stopMoving()

                uav.moveTo(pglobalX, pglobalY)
                time.sleep(2)
                uav.land()
                VISITED_BASES += 1

            time.sleep(1)

        except(rospy.ROSException) as e:
            a = e
            #rospy.loginfo("Bases not found")


def goto_first_hanging_base():
    rospy.logwarn("Going over the first fixed base...\n")

    uav.moveTo(FIXED_BASE_1.x, FIXED_BASE_1.y, 4.85, 4.85)

    # foundLandingBase = findBase()

    uav.land()
    uav.landedBases.append([FIXED_BASE_1, 'fixa'])

    time.sleep(1.0)

    rospy.loginfo("Going up...")
    uav.moveAxisZ(findingBaseAltitude)


def goto_second_hanging_base():
    rospy.logwarn("Going over the second fixed base...\n")

    uav.moveTo(FIXED_BASE_2.x, FIXED_BASE_2.y, 4.85, 4.85)

    # foundLandingBase = findBase()

    uav.land()
    uav.landedBases.append([FIXED_BASE_2, 'fixa'])

    time.sleep(1.0)

    rospy.loginfo("Going up...")
    uav.moveAxisZ(findingBaseAltitude)

def goto_third_hanging_base():
    rospy.logwarn("Going over the third fixed base...\n")

    uav.moveTo(FIXED_BASE_3.x, FIXED_BASE_3.y, 4.85, 4.85)

    # foundLandingBase = findBase()

    uav.land()
    uav.landedBases.append([FIXED_BASE_3, 'fixa'])

    time.sleep(1.0)

    rospy.loginfo("Going up...")
    uav.moveAxisZ(findingBaseAltitude)

def goto_waypoints():
    global WAYPOINTS

    for waypoint in WAYPOINTS:
        rospy.loginfo("Going to the next waypoint...\n")

        try:
            clock = rospy.wait_for_message('/clock', Clock, timeout=1)

            if (clock.clock.secs >= 810):
                print("Tempo excedido. Voltando para base costeira")
                return
        except:
            pass

        uav.moveTo(waypoint[0], waypoint[1])

        foundLandingBase = findBase()

        if (foundLandingBase):
            print("Base detectada na posicao ({:.2f}, {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y))
            uav.landedBases.append([Point(uav.globalPosition.x, uav.globalPosition.y, 0), 'movel'])

            uav.movingBasesLanded += 1

            rospy.loginfo("Going up...")
            uav.moveAxisZ((findingBaseAltitude + 2) * 2)

        if (uav.movingBasesLanded == 3):
            rospy.logwarn("Todas bases moveis visitadas...")
            break

        if (len(uav.landedBases) == uav.maxLandBases):
            rospy.logwarn("All bases landed...")
            break

def goto_waypoints2():
    global WAYPOINTS
    global VISITED_BASES

    for waypoint in WAYPOINTS:
        rospy.loginfo("Going to the next waypoint...\n")
        uav.moveTo(waypoint[0], waypoint[1])

        time.sleep(1)


def goto_coastal_base():
    rospy.logwarn("Going over the coastal base...\n")

    uav.updateGlobalPosition()

    if (uav.globalPosition.x < -50 and uav.globalPosition.y < -30):
        uav.moveTo(-60, 30)
    elif (uav.globalPosition.x < 0 and uav.globalPosition.x >= -50 and uav.globalPosition.y < -50):
        uav.moveTo(10, -65)

    uav.moveTo(COASTAL_BASE.x, COASTAL_BASE.y, thresholdX = 4.85, thresholdY = 4.85)

    # foundLandingBase = findBase()

    uav.landFinal()

def mission_planner():
    global findingBaseAltitude

    uav.updateGlobalPosition()

    rospy.logwarn("Iniciando...\n")
    rospy.logwarn("Posicao relativa:\t{:.2f} {:.2f} {:.2f}".format(uav.relativePosition.x, uav.relativePosition.y, uav.relativePosition.z))
    rospy.logwarn("Posicao global:\t{:.2f} {:.2f} {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y, uav.globalPosition.z))

    rospy.loginfo("Going up...\n")
    uav.moveAxisZ(4.5)

    goto_first_hanging_base()

    uav.moveAxisZ(10.0)

    goto_second_hanging_base()

    uav.moveAxisZ(3.0)

    uav.moveTo(-56, -21)

    goto_third_hanging_base()

    tubeCenter = get_tube_center()

    uav.moveTo(tubeCenter.x, tubeCenter.y, 4.75, 4.75)
    time.sleep(1)
    print('\n' + COLORS.BOLD + COLORS.GREEN + "Tubo localizado. Posicao central: ({:.2f}, {:.2f}, {:.2f})".format(tubeCenter.x, tubeCenter.y, tubeCenter.z) + COLORS.END + COLORS.END + '\n')
    time.sleep(1.5)

    uav.moveTo(-56, -21)

    uav.moveAxisZ(1.0)

    goto_waypoints()

    uav.moveAxisZ(2)

    goto_coastal_base()

def mission_planner2():
    global findingBaseAltitude

    uav.updateGlobalPosition()

    rospy.logwarn("Iniciando...\n")
    rospy.logwarn("Posicao relativa:\t{:.2f} {:.2f} {:.2f}".format(uav.relativePosition.x, uav.relativePosition.y, uav.relativePosition.z))
    rospy.logwarn("Posicao global:\t{:.2f} {:.2f} {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y, uav.globalPosition.z))

    rospy.loginfo("Finding the right altitude...\n")
    uav.moveAxisZ(10.0)

    thread = Thread(target= findBase2)
    thread.start()
    goto_waypoints2()

if __name__ == "__main__":

    time.sleep(9.0)

    rospy.init_node("mapping_task_node", anonymous=False)

    call_safety_area(False)
    call_min_height(0.0)

    rospy.set_param('/uav1/uav_manager/min_height_checking/enabled', False)

    # rospy.Subscriber("/uav1/garmin/range", Float64Stamped, callback_height)
    rospy.Subscriber("/uav1/odometry/height", Float64Stamped, callback_height)

    mission_planner()

    rospy.spin()
