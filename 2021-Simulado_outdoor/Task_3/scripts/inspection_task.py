#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
from std_srvs.srv import *
from mrs_msgs.msg import Float64Stamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from math import cos

# Coordenadas do gazebo
START_TUBE_EDGE = Point(-50, -25, 10)   
FINAL_TUBE_EDGE = Point(-50, -44.5, 10)
COASTAL_BASE = Point(10, 90, 0)
FIXED_BASE_1 = Point(45, 10, 2.6)          # Base do pier 
FIXED_BASE_2 = Point(-19.4, -21.28, 13.6)  # Base mais alta
FIXED_BASE_3 = Point(-54.15, -35.1, 9.85)     # Base perto do cano
MOVING_BASE_1 = Point(60.0, 0.0, 1.7)
MOVING_BASE_2 = Point(30, -55, 1.7)
MOVING_BASE_3 = Point(-20.0, 30.0, 1.7)

# MOVING_BASE_1 -> FIXED_BASE_1 -> MOVING_BASE_2 -> MOVING_BASE_3 -> FIXED_BASE_2 -> FIXED_BASE_3 -> COASTAL_BASE

sensorIsInRange = Bool
currentHeight = 0
findingBaseHeight = 7.0
hoverHeight = 1.5
adjustingPosition = False
distanciaFocal = 215.68

publisherToggleDetection = rospy.Publisher("/gas_detector/activate", Bool, queue_size=10)

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

class Uav:
    def __init__(self) -> None:
        self.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.relativePosition = Point(0.0, 0.0, 0.0)
        self.globalPosition = Point(10.0, 90.0, 0.71)
        self.globalStartPosition = Point(10.0, 90.0, 0.71)
        self.landedBases = [COASTAL_BASE] # Coastal base precisa ser incluida pois o drone pode detectar ela e tente pousar
        self.maxLandBases = 7 # 1 coastal + 3 fixed bases + 3 moving bases 

    def getRelativePosition(self):
        pose = rospy.wait_for_message("/uav1/mavros/local_position/pose", PoseStamped, timeout=6.0)
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

            debugMessage = "Posicao do alvo\t {:.2f} {:.2f}\nDistancia alvo\t {:.2f} {:.2f}".format(targetX, targetY, distanceToTargetX, distanceToTargetY);

            print(COLORS.BOLD + COLORS.WARNING + debugMessage + COLORS.END + COLORS.END + '\n')

            # Verifica se o drone está parado no mundo
            if (abs(lastDistanceX - distanceToTargetX) < 0.5 and abs(lastDistanceY - distanceToTargetY) < 0.5):
                print(COLORS.BOLD + COLORS.FAIL + "UAV parado. Movendo novamente..." + COLORS.END + COLORS.END + '\n')
                call_goto_relative(distanceToTargetX, distanceToTargetY, 0.0, 0.0)
                time.sleep(3.5)
            
            lastDistanceX = distanceToTargetX
            lastDistanceY = distanceToTargetY
        
        print(COLORS.BOLD + COLORS.WARNING + "Chegou no alvo" + COLORS.END + COLORS.END + '\n')
        time.sleep(1)

    def moveAxisZ(self, relativeZ):
        global currentHeight, hoverHeight

        self.updateGlobalPosition()

        targetZ = relativeZ + self.globalPosition.z

        call_goto_relative(0, 0, relativeZ, 0)

        time.sleep(2)

        while abs(relativeZ) > 0.35:
            time.sleep(0.5) 

            self.updateGlobalPosition()

            debugMessage = "Distancia\t {:.2f}".format(relativeZ)

            print(COLORS.BOLD + COLORS.BLUE + debugMessage + COLORS.END + COLORS.END + '\n')

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

        while currentHeight > 0.29:
            rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
            self.moveAxisZ(-currentHeight)
            time.sleep(currentHeight - (currentHeight * 0.7))

        rospy.loginfo("Land finished...")

    def landFinal(self):
        print("\n" + COLORS.BOLD + COLORS.WARNING + "Landing..." + COLORS.END + COLORS.END + "\n" )

        global currentHeight
        print(COLORS.BOLD + COLORS.WARNING + "Altura:\t{:.2f}".format(currentHeight) + COLORS.END + COLORS.END + '\n')
        if (currentHeight >= 1):
            heightToGoDown = currentHeight - 0.4
            self.moveAxisZ(-heightToGoDown)
            time.sleep(heightToGoDown - (heightToGoDown * 0.3))

        call_uav_manager_land()

    def hover(self, hoverHeight = 1.0):
        print("\n" + COLORS.BOLD + COLORS.WARNING + "Hovering..." + COLORS.END + COLORS.END + "\n" )

        global currentHeight

        heightToMove = hoverHeight - currentHeight

        self.moveAxisZ(heightToMove)

        if (heightToMove < 0):
            heightToMove *= -1

        time.sleep(heightToMove)

        while abs(currentHeight - hoverHeight) > 0.15:
            heightToMove = hoverHeight - currentHeight
            self.moveAxisZ(heightToMove)

            if (heightToMove < 0):
                heightToMove *= -1

            time.sleep(heightToMove)

    def alreadyLandedBase(self):
        for base in self.landedBases:
            if abs(self.globalPosition.x - base.x) < 1.0 and abs(self.globalPosition.y - base.y) < 1.0:
                return True

        return False

uav = Uav()


def estimateBasePosition(dxPixel, dyPixel, row, pitch):
    relativeX = (dxPixel * (uav.globalPosition.z + 1.4)) / (cos(row) * distanciaFocal)
    relativeY = (dyPixel * (uav.globalPosition.z + 1.4)) / (cos(pitch) * distanciaFocal)

    uav.updateGlobalPosition()

    globalX = uav.globalPosition.x + relativeX
    globalY = uav.globalPosition.y + relativeY

    return globalX, globalY

def ajustAndDetect():

    delta_x = 0
    delta_y = 0
    delta_z = -0.3

    width = 50
    height = 50

    firstMove = True

    success = False
    PX_THRESHOLD = width/2
    PY_THRESHOLD = height/2
    ADJUSTED_STEP = 0.5

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
            distancePixels = rospy.wait_for_message("/base_detector/px_py", Float64MultiArray, 3)
        except(rospy.ROSException) as e:
            if (uav.globalPosition.z - 1.7) > 8 or (uav.globalPosition.z - 1.7 < 0):
                print("Reajustando posicao")
                uav.moveAxisZ((uav.globalPosition.z - 1.7) + 5)

            print("Falha ao detectar base")
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
                PX_THRESHOLD = width/2.5
            elif (width < 55):
                PX_THRESHOLD = width/3.8
            else:
                PX_THRESHOLD = width/4.8

            if (height < 43):
                PY_THRESHOLD = height/2.5
            elif (height < 55):
                PY_THRESHOLD = height/3.8
            else:
                PY_THRESHOLD = height/4.8

            print("\n" + COLORS.BOLD + COLORS.GREEN + "Threshold ({:.2f}, {:.2f})".format(PX_THRESHOLD, PY_THRESHOLD) + COLORS.END + COLORS.END)
            print(COLORS.BOLD + COLORS.CYAN + "Width (w: {:.1f}, h: {:.1f})".format(width, height) + COLORS.END + COLORS.END)

        except(rospy.ROSException) as e:
            rospy.loginfo("Using default baseSize")
            break

        print(COLORS.BOLD + COLORS.BLUE + "Distances (dx: {:.1f}, dy: {:.1f})".format(distancePixels.data[1], distancePixels.data[0]) + COLORS.END + COLORS.END)

        if distancePixels.data[0] > PY_THRESHOLD:
            delta_y = -ADJUSTED_STEP
        elif distancePixels.data[0] < -PY_THRESHOLD:
            delta_y = ADJUSTED_STEP
        else:
            delta_y = 0.0

        if distancePixels.data[1] + 40 > PX_THRESHOLD:
            delta_x = ADJUSTED_STEP
        elif distancePixels.data[1] + 40 < -PX_THRESHOLD:
            delta_x = -ADJUSTED_STEP
        else:
            delta_x = 0.0

        if (width >= 80 and height >= 80):
            delta_z = 0

        print(COLORS.BOLD + COLORS.GREEN + "Moving: ({:.2f}, {:.2f}, {:.2f})".format(delta_x, delta_y, delta_z) + COLORS.END + COLORS.END + "\n" )

        call_goto_relative(delta_x, delta_y, delta_z, 0.0)
        time.sleep(1.7)

        if (abs(distancePixels.data[0]) < PY_THRESHOLD) and (abs(distancePixels.data[1] + 50) < PX_THRESHOLD):
            print("Going down...")

            heightToGoDown = currentHeight - (currentHeight * 0.55)

            if (currentHeight - heightToGoDown < 2):
                heightToGoDown = (currentHeight - 2) + 0.15

            call_goto_relative(0, 0, -heightToGoDown, 0.0)
            time.sleep(2.5)

        if (biggestHeight > 140 and biggestWidth > 140 and currentHeight < 2.2):
            print("Over the base")

            print("Starting detecion")
            publisherToggleDetection.publish(True)

            print('\n' + COLORS.BOLD + COLORS.WARNING + "Buscando marcador..." + COLORS.END + COLORS.END + '\n')
            detection = rospy.wait_for_message("/gas_detector/detected", Bool)

            if (detection.data):
                print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado! Realizando leitura..." + COLORS.END + COLORS.END + '\n')

                array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
                array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)

                # Para a detecção
                publisherToggleDetection.publish(False)

                call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
                totalSensorsDetected += 1
            else:
                print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

                # Para a detecção
                publisherToggleDetection.publish(False)


            success = True
            break


    return success

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
            elif (width < 90):
                PX_THRESHOLD = width/5.2
            else:
                PX_THRESHOLD = width/5.6

            if (height < 43):
                PY_THRESHOLD = height/2
            elif (height < 55):
                PY_THRESHOLD = height/3.5
            elif (height < 90):
                PY_THRESHOLD = height/5.2
            else:
                PY_THRESHOLD = height/5.6

            print("\n" + COLORS.BOLD + COLORS.GREEN + "Threshold ({:.2f}, {:.2f})".format(PX_THRESHOLD, PY_THRESHOLD) + COLORS.END + COLORS.END)
            print(COLORS.BOLD + COLORS.CYAN + "Width (w: {:.1f}, h: {:.1f})".format(width, height) + COLORS.END + COLORS.END)

        except(rospy.ROSException) as e:
            rospy.loginfo("Using default baseSize")
            break

        print(COLORS.BOLD + COLORS.BLUE + "Distances (dx: {:.1f}, dy: {:.1f})".format(distancePixels.data[1], distancePixels.data[0]) + COLORS.END + COLORS.END)

        if firstMove:
            globalX, globalY = estimateBasePosition(distancePixels.data[1], -distancePixels.data[0], euler[1], euler[0])

            print(COLORS.BOLD + COLORS.BLUE + "Estimativa posicao (x: {:.1f}, y: {:.1f})".format(globalX, globalY) + COLORS.END + COLORS.END)

            uav.moveTo(globalX, globalY, 4.8, 4.8)
            firstMove = False
            time.sleep(1)
            continue

        if distancePixels.data[0] > PY_THRESHOLD:
            delta_y = -ADJUSTED_STEP
        elif distancePixels.data[0] < -PY_THRESHOLD:
            delta_y = ADJUSTED_STEP
        else:
            delta_y = 0.0

        if (distancePixels.data[1] + 15) > PX_THRESHOLD:
            delta_x = ADJUSTED_STEP
        elif (distancePixels.data[1] + 15) < -PX_THRESHOLD:
            delta_x = -ADJUSTED_STEP
        else:
            delta_x = 0.0

        if (width >= 80 and height >= 80):
            delta_z = 0

        print(COLORS.BOLD + COLORS.GREEN + "Moving: ({:.2f}, {:.2f}, {:.2f})".format(delta_x, delta_y, delta_z) + COLORS.END + COLORS.END + "\n" )

        call_goto_relative(delta_x, delta_y, delta_z, 0.0)
        time.sleep(1.7)

        if (abs(distancePixels.data[0]) < PY_THRESHOLD) and (abs(distancePixels.data[1] + 15) < PX_THRESHOLD):
            print("Going down...")

            heightToGoDown = currentHeight - (currentHeight * 0.55)

            if (currentHeight - heightToGoDown < 1.8):
                heightToGoDown = (currentHeight - 1.8) + 0.3

            call_goto_relative(0, 0, -heightToGoDown, 0.0)
            time.sleep(2.5)

        if (biggestHeight > 150 and biggestWidth > 150 and currentHeight < 1.8):
            print("Over the base")

            success = True
            break

    return success


def get_tube_center():
    centerX = (START_TUBE_EDGE.x + FINAL_TUBE_EDGE.x) / 2
    centerY = (START_TUBE_EDGE.y + FINAL_TUBE_EDGE.y) / 2

    return Point(centerX, centerY, START_TUBE_EDGE.z)

def call_uav_manager_land():
    try:
        service = rospy.ServiceProxy("/uav1/uav_manager/land", Trigger)
        rospy.wait_for_service("/uav1/uav_manager/land")

        service()
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_goto_relative(x, y, z, rxy):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/goto_relative", Vec4)
        rospy.wait_for_service("/uav1/control_manager/goto_relative")

        service([x,y,z,rxy])
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s') % e

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
        rospy.wait_for_service("/uav1/mavros/set_mode")

        print(service(mode_ID, mode))
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s') % e 

def call_arming(value):
    try:
        service = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav1/mavros/cmd/arming")

        print(service(value))
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s') % e

def call_safety_area(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/use_safety_area", SetBool)
        rospy.wait_for_service("/uav1/control_manager/use_safety_area")

        print(service(value))
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s') % e

def call_min_height(value):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/set_min_height", Float64Srv)
        rospy.wait_for_service("/uav1/control_manager/set_min_height")

        print(service(value))
        
    except rospy.ServiceException as e:
        print ('Service call failed: %s') % e


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


def call_console_signal(gas_right, ajuste_right, gas_value, ajuste_value):
    # CHAMA O SINAL EXTERNO PARA O VALOR DO GAS
    if (gas_right):
        print(COLORS.BOLD + COLORS.GREEN + "VALOR DE GAS DENTRO DOS LIMITES!  {}".format(gas_value) + COLORS.END + COLORS.END + '\n')
        # call_external_signal(array.data[0], 25)
    else:
        print(COLORS.BOLD + COLORS.WARNING + "VALOR DE GAS FORA DOS LIMITES!  {}".format(gas_value) + COLORS.END + COLORS.END + '\n')

    # CHAMA O SINAL EXTERNO PARA O VALOR DO AJUSTE DE ZERO
    if (ajuste_right):
        print(COLORS.BOLD + COLORS.GREEN + "VALOR DO AJUSTE DE ZERO DENTRO DOS LIMITES!  {}".format(ajuste_value) + COLORS.END + COLORS.END + '\n')
        # call_external_signal(array.data[0], 25)
    else:
        print(COLORS.BOLD + COLORS.WARNING + "VALOR DO AJUSTE DE ZERO FORA DOS LIMITES!  {}".format(ajuste_value) + COLORS.END + COLORS.END + '\n')
    
    i = 0
    print("\nEsperando 5 segundos...")
    while(i<5):
        i += 1
        print(i)
        time.sleep(1)
    print("\n")


def hover_base():
    if (currentHeight > 1.6):
        call_goto_relative(0.0, 0.0, -0.8, 0.0)
        time.sleep(2.2)

    while (currentHeight > 0.8):
        call_goto_relative(0.0, 0.0, -0.2, 0.0)
        time.sleep(0.4)


def callback_height(data):
    global currentHeight
    currentHeight = data.value

def goto_first_edge():
    uav.moveTo(START_TUBE_EDGE.x, START_TUBE_EDGE.y, thresholdX=4.5, thresholdY=4.5)


def goto_final_edge():
    uav.moveTo(FINAL_TUBE_EDGE.x, FINAL_TUBE_EDGE.y, thresholdX=4.5, thresholdY=4.5)

def goto_first_hanging_base():
    print("Going over the first fixed base...\n")

    uav.moveTo(FIXED_BASE_1.x, FIXED_BASE_1.y, 4.85, 4.85)


def goto_second_hanging_base():
    print("Going over the second fixed base...\n")

    uav.moveTo(FIXED_BASE_2.x, FIXED_BASE_2.y, 4.9, 4.9)

def goto_third_hanging_base():
    print("Going over the third fixed base...\n")

    uav.moveTo(FIXED_BASE_3.x, FIXED_BASE_3.y, 4.85, 4.85)


def goto_bases(base: Point):
    global findingBaseHeight, currentHeight

    print("Going over next base...\n")

    uav.updateGlobalPosition()

    baseRelativeZ = base.z - uav.globalPosition.z + findingBaseHeight

    uav.moveAxisZ(baseRelativeZ)
    uav.moveTo(base.x, base.y, thresholdX=4.5, thresholdY=4.5)

    time.sleep(1)


def goto_coastal_base():
    print("Going over the coastal base...\n")

    uav.moveTo(COASTAL_BASE.x, COASTAL_BASE.y, thresholdX = 4.85, thresholdY = 4.85)


def is_in_range(array):
    global sensorIsInRange
    sensorIsInRange = array.data[0]


def mission_planner():
    global findingBaseHeight

    totalSensorsDetected = 0
    foundMovingSensor = False

    uav.updateGlobalPosition()

    print("Posicao relativa:\t{:.2f} {:.2f} {:.2f}\n".format(uav.relativePosition.x, uav.relativePosition.y, uav.relativePosition.z))
    print("Posicao global:\t{:.2f} {:.2f} {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y, uav.globalPosition.z))

    # MOVING_BASE_1 -> MOVING_BASE_2 -> MOVING_BASE_3 -> FIXED_BASE_2 -> FIXED_BASE_3 -> COASTAL_BASE

    print("Going up...\n")
    uav.moveAxisZ(findingBaseHeight)

    uav.moveTo(MOVING_BASE_1.x, MOVING_BASE_1.y, 4.5, 4.5)

    uav.updateGlobalPosition()
    uav.moveAxisZ(5 - (uav.globalPosition.z + 1.7))

    foundBase = findBase()

    print("Starting detecion")
    publisherToggleDetection.publish(True)

    print('\n' + COLORS.BOLD + COLORS.WARNING + "Buscando marcador..." + COLORS.END + COLORS.END + '\n')
    detection = rospy.wait_for_message("/gas_detector/detected", Bool)

    if (detection.data):
        foundMovingSensor = True
        print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        # array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
        # array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)
        publisherToggleDetection.publish(False)

        i = 0
        print("\nEsperando 5 segundos...")
        while(i<5):
            i += 1
            print(i)
            time.sleep(1)
        print("\n")

        # call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
        totalSensorsDetected += 1
    else:
        publisherToggleDetection.publish(False)
        print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

    if foundMovingSensor:
        uav.moveAxisZ(findingBaseHeight * 2)
        # call_goto_relative(0, 0, findingBaseHeight * 2, 0)
        # time.sleep(findingBaseHeight + 5)
    else:
        uav.moveAxisZ(findingBaseHeight)
        # call_goto_relative(0, 0, findingBaseHeight, 0)
        # time.sleep(findingBaseHeight)

    if (not foundMovingSensor):
        uav.moveTo(MOVING_BASE_2.x, MOVING_BASE_2.y, 4.5, 4.5)

        uav.updateGlobalPosition()
        uav.moveAxisZ(5 - (uav.globalPosition.z + 1.7))

        foundBase = findBase()

        print("Starting detecion")
        publisherToggleDetection.publish(True)

        print('\n' + COLORS.BOLD + COLORS.WARNING + "Buscando marcador..." + COLORS.END + COLORS.END + '\n')
        detection = rospy.wait_for_message("/gas_detector/detected", Bool)

        if (detection.data):
            foundMovingSensor = True
            print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

            publisherToggleDetection.publish(False)

            i = 0
            print("\nEsperando 5 segundos...")
            while(i<5):
                i += 1
                print(i)
                time.sleep(1)
            print("\n")
            # array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
            # array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)

            # call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
            totalSensorsDetected += 1
        else:
            publisherToggleDetection.publish(False)
            print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        uav.moveAxisZ(findingBaseHeight * 2)
    
    if (not foundMovingSensor):
        uav.moveTo(MOVING_BASE_3.x, MOVING_BASE_3.y, 4.5, 4.5)

        uav.updateGlobalPosition()
        uav.moveAxisZ(5 - (uav.globalPosition.z + 1.7))

        foundBase = findBase()

        print("Starting detecion")
        publisherToggleDetection.publish(True)

        print('\n' + COLORS.BOLD + COLORS.WARNING + "Buscando marcador..." + COLORS.END + COLORS.END + '\n')
        detection = rospy.wait_for_message("/gas_detector/detected", Bool)

        if (detection.data):
            foundMovingSensor = True
            print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

            publisherToggleDetection.publish(False)

            i = 0
            print("\nEsperando 5 segundos...")
            while(i<5):
                i += 1
                print(i)
                time.sleep(1)
            print("\n")

            # array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
            # array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)

            # call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
            totalSensorsDetected += 1
        else:
            publisherToggleDetection.publish(False)
            print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        uav.moveAxisZ(findingBaseHeight * 2)
        # call_goto_relative(0, 0, findingBaseHeight * 2, 0)
        # time.sleep(findingBaseHeight + 6)

    goto_second_hanging_base()

    uav.hover(0.55)

    print("Starting detecion")
    publisherToggleDetection.publish(True)

    print('\n' + COLORS.BOLD + COLORS.WARNING + "Buscando marcador..." + COLORS.END + COLORS.END + '\n')
    detection = rospy.wait_for_message("/gas_detector/detected", Bool)

    if (detection.data):
        print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado! Realizando leitura..." + COLORS.END + COLORS.END + '\n')

        array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
        array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)

        # Para a detecção
        publisherToggleDetection.publish(False)

        call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
        totalSensorsDetected += 1
    else:
        print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        # Para a detecção
        publisherToggleDetection.publish(False)


    uav.moveAxisZ(findingBaseHeight)

    # Posicao pra nao colidir com a parede
    uav.moveTo(-50, -21, 4, 4)

    goto_third_hanging_base()

    uav.hover(0.55)

    print("Starting detecion")
    publisherToggleDetection.publish(True)

    detection = rospy.wait_for_message("/gas_detector/detected", Bool)

    if (detection.data):
        print('\n' + COLORS.BOLD + COLORS.GREEN + "Marcador de gas detectado! Realizando leitura..." + COLORS.END + COLORS.END + '\n')

        array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
        array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=25)

        # Para a detecção
        publisherToggleDetection.publish(False)

        call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
        totalSensorsDetected += 1
    else:
        print('\n' + COLORS.BOLD + COLORS.WARNING + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        # Para a detecção
        publisherToggleDetection.publish(False)

    uav.moveAxisZ(5)

    goto_coastal_base()

    uav.landFinal()

    return

    print("Starting detecion")
    publisherToggleDetection.publish(True)

    detection = rospy.wait_for_message("/gas_detector/detected", Bool)

    if (detection.data):
        print('\n' + COLORS.BOLD + COLORS.CYAN + "Marcador de gas detectado! Realizando leitura..." + COLORS.END + COLORS.END + '\n')

        array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
        rospy.logwarn("Waiting values...")
        array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=16)

        call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
        totalSensorsDetected += 1
    else:
        print('\n' + COLORS.BOLD + COLORS.CYAN + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

    # Para a detecção
    publisherToggleDetection.publish(False)
    
    uav.moveAxisZ(findingBaseHeight)
    rospy.loginfo("Going up...")


    totalSensorsDetected = 0
    for movingBase in [MOVING_BASE_1, MOVING_BASE_2, MOVING_BASE_3]:
        print('\n' + COLORS.BOLD + COLORS.CYAN + "Going to moving base x: {:.2f}  y: {:.2f}".format(movingBase.x, movingBase.y) + COLORS.END + COLORS.END + '\n')

        if (totalSensorsDetected >= 3):
            print('\n' + COLORS.BOLD + COLORS.CYAN + "Todos marcadores detectados..." + COLORS.END + COLORS.END + '\n')
            break

        goto_bases(movingBase)

        # uav.moveAxisZ(-8)
        # time.sleep(5)
        # uav.hover()

        # Inicia a detecção
        print("Starting detecion")
        publisherToggleDetection.publish(True)

        detection = rospy.wait_for_message("/gas_detector/detected", Bool)

        if (detection.data):
            print('\n' + COLORS.BOLD + COLORS.CYAN + "Marcador de gas detectado! Realizando leitura..." + COLORS.END + COLORS.END + '\n')

            array = rospy.wait_for_message("/gas_detector/in_range", ByteMultiArray, timeout=60)
            rospy.logwarn("Waiting values...")
            array_values = rospy.wait_for_message("/gas_detector/values", Int16MultiArray, timeout=16)

            call_console_signal(array.data[0], array.data[1], array_values.data[0], array_values.data[1])
            totalSensorsDetected += 1
        else:
            print('\n' + COLORS.BOLD + COLORS.CYAN + "Nenhum marcador de gas detectado!" + COLORS.END + COLORS.END + '\n')

        # Para a detecção
        publisherToggleDetection.publish(False)
        
        uav.moveAxisZ(findingBaseHeight)
        rospy.loginfo("Going up...")

    # BACK HOME
    goto_coastal_base()
    

if __name__ == "__main__": 

    time.sleep(10.0)

    rospy.init_node("inspection_task_main", anonymous=False)

    print('\n' + COLORS.BOLD + COLORS.GREEN + "Iniciando tarefa..." + COLORS.END + COLORS.END + '\n')

    call_safety_area(False)
    call_min_height(0.0)
    
    # rospy.Subscriber("/gas_detector/in_range", ByteMultiArray, is_in_range)
    rospy.Subscriber("/uav1/odometry/height", Float64Stamped, callback_height)
    # rospy.Subscriber("/base_detector/adjusting_position", Bool, callback_adjusting_position)

    # r = rospy.Rate(5) # 10hz
    # while not rospy.is_shutdown():
    mission_planner() 
        # r.sleep()
        # break
    rospy.spin()