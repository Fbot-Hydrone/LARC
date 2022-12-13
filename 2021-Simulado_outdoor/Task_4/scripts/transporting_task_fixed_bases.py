#! /usr/bin/env python3
import time
import rospy
from std_msgs.msg import *
from mrs_msgs.srv import *
from std_srvs.srv import *
from mrs_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from math import cos
from geometry_msgs.msg import *
# from gazebo_ros_link_attacher.srv import *
from tf.transformations import euler_from_quaternion

findingBaseHeight = 7.0
distanciaFocal = 215.68

# Coordenadas do gazebo
COASTAL_BASE = Point(10, 90, 0)
FIXED_BASE_1 = Point(45, 10, 2.6)          # Base do pier 
FIXED_BASE_2 = Point(-19.4, -21.28, 13.6)  # Base mais alta
FIXED_BASE_3 = Point(-54.15, -35.1, 9.85)  # Base perto do cano
MOVING_BASE_1 = Point(65.0, 0.0, 1.7)      # Movel atras do pier
MOVING_BASE_2 = Point(38, -55, 1.7)        # Movel aberta para mar
MOVING_BASE_3 = Point(-18.0, 30.0, 1.7)    # Base perto da costal

BASES_TO_CHECK = [MOVING_BASE_1, MOVING_BASE_2, MOVING_BASE_3]

BASES =	{
  "A": MOVING_BASE_3,
  "B": MOVING_BASE_1,
  "C": MOVING_BASE_2,
  "D": FIXED_BASE_2,
  "E": FIXED_BASE_3
}


# BASES_TO_CHECK = [HANGING_BASE_1, HANGING_BASE_2, LANDING_BASES[0], LANDING_BASES[1], LANDING_BASES[2]]

# BASES =	{
#   "A": LANDING_BASES[0],
#   "B": LANDING_BASES[1],
#   "C": LANDING_BASES[2],
#   "E": HANGING_BASE_1,
#   "D": HANGING_BASE_2
# }

DICT_LETTERS_NUMBERS = {
    1: "A",
    2: "B",
    3: "C",
    4: "D",
    5: "E"
}

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

base_letter = ""
last_base_letter = ""
current_height = 0
DELTA = 0.2
DX_DY_THRESHOLD = 28.0
ADJUST_QRCODE_STEP = 0.05
total_bases_landed = 0

currentHeight = 0
findingBaseAltitude = 5.0

position = PoseStamped()
detected_qrcode = Int8()

pub_activate_detection = rospy.Publisher("/block_detector/activate", Bool, queue_size=10)

class Uav:
    def __init__(self) -> None:
        self.stopMovingFlag = False
        self.isMoving = False
        self.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.lastPosition = Point(0.0, 0.0, 0.0)
        self.relativePosition = Point(0.0, 0.0, 0.0)
        self.globalPosition = Point(10.0, 90.0, 0.71)
        self.globalStartPosition = Point(10.0, 90.0, 0.71)
        self.landedBases = [COASTAL_BASE] # Coastal base precisa ser incluida pois o drone pode detectar ela e tente pousar
        self.maxLandBases = 7 # 1 coastal + 3 fixed bases + 3 moving bases

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


    def stopMoving(self):
        if (not self.isMoving):
            return

        self.stopMovingFlag = True

        rospy.logwarn("Stoping...")

        time.sleep(1)
        
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

    def go_down(self):
        global currentHeight
        rospy.logwarn("Altura:\t{:.2f}".format(currentHeight))
        if (currentHeight >= 1.25):
            heightToGoDown = currentHeight - 0.5
            uav.moveAxisZ(-heightToGoDown)
            time.sleep(heightToGoDown - (heightToGoDown * 0.6))

        rospy.loginfo("Hovering over base...")

    def alreadyLandedBase(self, x = None, y = None):
        self.updateGlobalPosition()

        if (not x and not y):
            x = self.globalPosition.x
            y = self.globalPosition.y

        for base in self.landedBases:
            if abs(base.x - x) < 7.0 and abs(base.y - y) < 8.0:
                return True

        return False

uav = Uav()

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


def call_uav_manager_land():
    try:
        service = rospy.ServiceProxy("/uav1/uav_manager/land", Trigger)
        rospy.wait_for_service("/uav1/uav_manager/land")

        print(service())

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)


def set_kqxy(value):
    rospy.set_param('/uav1/control_manager/mpc_controller/kqxy', value)

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

def call_change_controller(controller):
    try:
        service = rospy.ServiceProxy("/uav1/control_manager/switch_controller", String)
        rospy.wait_for_service("/uav1/control_manager/switch_controller")

        rospy.loginfo("Switching controller to %s", controller)
        print(service(controller))

    except rospy.ServiceException as e:
        print ('Service call failed: %s' % e)

def call_external_signal():
    global detected_bad

    call_goto_relative(0.0, 0.0, 0.5, 0.0)
    time.sleep(5)

    call_goto_relative(0.0, 0.0, -0.5, 0.0)
    time.sleep(11)

    detected_bad = False

def goto_hover_from_base():
    while (current_height < 0.5):
        call_goto_relative(0.0, 0.0, 0.15, 0.0)
        time.sleep(0.5)


def goto_hover_base():
    while (current_height > 0.57):
        call_goto_relative(0.0, 0.0, -0.15, 0.0)
        time.sleep(0.3)

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

def goto_first_hanging_base():
    rospy.logwarn("Going over the first fixed base...\n")

    uav.moveTo(FIXED_BASE_1.x, FIXED_BASE_1.y, 4.85, 4.85)


def goto_second_hanging_base():
    rospy.logwarn("Going over the second fixed base...\n")

    uav.moveTo(FIXED_BASE_2.x, FIXED_BASE_2.y, 4.85, 4.85)

    # foundLandingBase = findBase()

def goto_third_hanging_base():
    rospy.logwarn("Going over the third fixed base...\n")

    uav.moveTo(FIXED_BASE_3.x, FIXED_BASE_3.y, 4.85, 4.85)

    # foundLandingBase = findBase()


def goto_coastal_base():
    rospy.logwarn("Going over the coastal base...\n")

    uav.moveTo(COASTAL_BASE.x, COASTAL_BASE.y, thresholdX = 4.1, thresholdY = 4.1)

    # foundLandingBase = findBase()

    uav.land()

def callback_height(data):
    global current_height
    current_height = data.value

def callback_qrcode_detection(data):
    global detected_qrcode
    detected_qrcode = data

def callback_block_name(data):
    rospy.logdebug("asiduhaui")
    # rospy.logwarn("callback said: %s", data.data)

def callback_height(data):
    global currentHeight
    currentHeight = data.value

def estimateBasePosition(dxPixel, dyPixel, row, pitch):
    relativeX = (dxPixel * (uav.globalPosition.z + 1.4)) / (cos(row) * distanciaFocal)
    relativeY = (dyPixel * (uav.globalPosition.z + 1.4)) / (cos(pitch) * distanciaFocal)

    uav.updateGlobalPosition()

    globalX = uav.globalPosition.x + relativeX
    globalY = uav.globalPosition.y + relativeY

    return globalX, globalY


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

        if (distancePixels.data[1] + 25) > PX_THRESHOLD:
            delta_x = ADJUSTED_STEP
        elif (distancePixels.data[1] + 25) < -PX_THRESHOLD:
            delta_x = -ADJUSTED_STEP
        else:
            delta_x = 0.0

        if (width >= 80 and height >= 80):
            delta_z = 0

        # print(COLORS.BOLD + COLORS.GREEN + "Moving: ({:.2f}, {:.2f}, {:.2f}".format(delta_x, delta_y, delta_z) + COLORS.END + COLORS.END + "\n" )

        call_goto_relative(delta_x, delta_y, delta_z, 0.0)
        time.sleep(1.7)

        if (abs(distancePixels.data[0]) < PY_THRESHOLD) and (abs(distancePixels.data[1] + 25) < PX_THRESHOLD):
            print("Going down...")

            heightToGoDown = currentHeight - (currentHeight * 0.55)

            if (currentHeight - heightToGoDown < 0.85):
                heightToGoDown = (currentHeight - 0.85) + 0.3

            call_goto_relative(0, 0, -heightToGoDown, 0.0)
            time.sleep(2.5)

        if (biggestHeight > 150 and biggestWidth > 150 and currentHeight < 0.85):
            print("Over the base")

            success = True
            break

    return success

def callback_block_name_number(data):
    global base_letter
    global last_base_letter
    last_base_letter = base_letter
    base_letter = data
    # rospy.logwarn("BLOCK %s", base_letter)

def mission_planner():
    global findingBaseAltitude
    global base_letter
    global BASES
    global total_bases_landed
    global last_base_letter

    uav.updateGlobalPosition()

    rospy.logwarn("Iniciando...\n")
    rospy.logwarn("Posicao relativa:\t{:.2f} {:.2f} {:.2f}".format(uav.relativePosition.x, uav.relativePosition.y, uav.relativePosition.z))
    rospy.logwarn("Posicao global:\t{:.2f} {:.2f} {:.2f}\n".format(uav.globalPosition.x, uav.globalPosition.y, uav.globalPosition.z))

    print("Going up...\n")
    uav.moveAxisZ(findingBaseHeight)
    
    time.sleep(1)

    goto_second_hanging_base()

    call_change_controller("MpcController")

    uav.hover(0.53)

    pub_activate_detection.publish(True)

    try:
        found = rospy.wait_for_message('/block_detector/found_qrcode', Bool, timeout=8)    
    except(rospy.ROSException) as e:
        print('QRCODE nao encontrado')

    pub_activate_detection.publish(False)

    uav.moveAxisZ(findingBaseHeight)

    call_change_controller("Se3Controller")
    
    # Posicao pra nao colidir com a parede
    uav.moveTo(-50, -21, 4, 4)

    goto_third_hanging_base()

    call_change_controller("MpcController")

    uav.hover(0.53)

    pub_activate_detection.publish(True)

    try:
        found = rospy.wait_for_message('/block_detector/found_qrcode', Bool, timeout=8)    
    except(rospy.ROSException) as e:
        print('QRCODE nao encontrado')

    pub_activate_detection.publish(False)

    uav.moveAxisZ(5)

    call_change_controller("Se3Controller")

    goto_coastal_base()

    call_change_controller("MpcController")

    uav.landFinal()

if __name__ == "__main__":

    time.sleep(10.0)

    rospy.init_node("transporting_task_node", anonymous=False)
    call_safety_area(False)
    call_min_height(0.0)

    rospy.set_param('/uav1/uav_manager/min_height_checking/enabled', False)

    # rospy.Subscriber("/block_detector/detection", String, callback_block_name)
    rospy.Subscriber("/block_detector/detection_number", Int8, callback_block_name_number)
    rospy.Subscriber("/uav1/odometry/height", Float64Stamped, callback_height)
    # rospy.Subscriber("/block_detector/detection_number", Int8, callback_qrcode_detection)

    mission_planner()

    rospy.spin()
