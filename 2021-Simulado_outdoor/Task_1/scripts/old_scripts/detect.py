import landing_mark_class as lm
import cv2
import time, os, math
import matplotlib.pyplot as plt
import numpy as np

USE_CAMERA = True
USE_VIDEO = False
USE_IMAGE = False

# get image
if USE_CAMERA:
	cap = cv2.VideoCapture(0)
elif USE_VIDEO:
	cap = cv2.VideoCapture('teste.mp4')
else:
	imagefile = "./videos/1.png"
			
mark = lm.LandingMark()

######################################################################
# Main loop
######################################################################
t = time.clock()
Z = []
Time = []

plt.figure()
plt.ion()

i = 0

#database_dir = "./Dataset/"
#for image_file in os.listdir("./Dataset"):
while True:

	#dt = abs(time.clock() - t)
	#t = time.clock()	
	#print round(1.0/dt, 2), "Hz"

	# get image
	if USE_CAMERA or USE_VIDEO:
		# Capture frame-by-frame
		ret, img = cap.read()
	else:
		# Read image
		img = cv2.imread(database_dir + image_file) # queryImage

	# image para a classe
	mark.setImage(img, 0.)
	
	# get reference frame
	R, T, success = mark.getRefFrame()
	
	#print success
	(x, y, z) = T
	#(roll, pitch, yaw) = np.rad2deg(R)
	
	'''rvec_matrix = cv2.Rodrigues(R)[0]
	proj_matrix = np.hstack((rvec_matrix, T))
	eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)[6]
	pitch, yaw, roll = [math.radians(_) for _ in eulerAngles]
	pitch = math.degrees(math.asin(math.sin(pitch)))
	roll = -math.degrees(math.asin(math.sin(roll)))
	yaw = math.degrees(math.asin(math.sin(yaw)))
	
	#print "x = ", round(x,1), "\ty = ", round(y,1), "\tz = ", round(z,1)
	#print "roll = ", round(roll,1), "\tpitch = ", round(pitch,1), "\tyaw = ", round(yaw,1)'''
		
	# Z.append(z)
	# Time.append(time.clock()-t)
	
	# i = i+1
	# if not (i%20):
	# 	plt.plot(Time, Z, 'ro-')
	# 	plt.show()
	# 	plt.pause(.001)

	print(x,y,z)
	
	# show
	mark.show()
	
	# ESC to finish
	k = cv2.waitKey(33) & 0xFF
	if k == 27:
		break
