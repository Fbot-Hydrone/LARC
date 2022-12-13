import cv2
import numpy as np

######################################################################
# GLOBAIS
######################################################################
# limiares de saturacao e valor
MINSAT = 50
MAXSAT = 255
MINVAL = 50
MAXVAL = 255

# limiares da cor amarela
YELLOW = 25
DYELLOW = 20
MINYELLOW = YELLOW - DYELLOW
MAXYELLOW = YELLOW + DYELLOW

# TESTE NICOLAS
MINYELLOW = 20
MAXYELLOW = 45

# limiares da cor azul
BLUE = 110
DBLUE = 25
# MINBLUE = BLUE - DBLUE
# MAXBLUE = BLUE + DBLUE

# TESTE NICOLAS
MINBLUE = 95
MAXBLUE = 135

# parametros de filtros
GAUSSIAN_FILTER = 3
KERNEL_RESOLUTION = 7

# dimensoes da base real
ARESTA = 500.0 # aresta da base (em mm)
RAIO = 200.0 # raio do centro da base 
RESOLUTION = 50
	
#############################################################################
# classe para detectar landmarks para o desafio Petrobras
#############################################################################
class LandingMark:
	#########################################################################
	# construtor
	#########################################################################
	def __init__(self):
	
		# cria pontos do objeto 3D para o PNP
		#self.model3DRect()
		self.model3DEllipse()
		
		# reference frame
		self.rot_vec = np.zeros((4,1))
		self.trans_vec = np.zeros((3,1))
		
		# posicao relativa
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
	
	######################################################################
	# Camera internals
	######################################################################
	def CamParam(self, img):
		
		# tamanho da imagem
		self.size = img.shape
		
		# distancia focal
		self.focal_length = self.size[1]
		
		# centro otico
		self.center = (self.size[1]/2, self.size[0]/2)
		
		# Matriz da camera
		self.camera_matrix = np.array(	[[self.focal_length, 0, self.center[0]],
		                     		[0, self.focal_length, self.center[1]],
		                     		[0, 0, 1]], dtype = "double")
		                     		
		# distorcoes da lente
		self.dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
		#print "Camera Matrix :\n {0}".format(camera_matrix)
		
	######################################################################
	# fornece imagem a ser processada. Fator redimensiona a image
	######################################################################
	def setImage(self, img, fator = 0):
	
		# filtro gaussiano
		img = cv2.GaussianBlur(img, (GAUSSIAN_FILTER, GAUSSIAN_FILTER), 0)
	
		# resize image
		if fator != 0:
			img = cv2.resize(img, None, fx = fator, fy = fator)
		
		# set camera internals from img	
		self.CamParam(img)
		
		# image a ser processada
		self.img = img
	
	######################################################################
	# processamento da imagem, antes do calculo do frame
	######################################################################
	def processImage(self):
		
		# convert self.img para HSV
		hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
		
		# kernel de convolucao
		kernel = np.ones((KERNEL_RESOLUTION, KERNEL_RESOLUTION), np.uint8)
		
		# pega o quadrado azul mais externo
		quadrado_azul = self.imlimiares(hsv, (MINBLUE, MINSAT, MINVAL), (MAXBLUE, MAXSAT, MAXVAL))
		hsv = cv2.bitwise_and(hsv, hsv, mask = quadrado_azul)
		
		# pega o circulo amarelo mais interno
		circulo_amarelo = self.imlimiares(hsv, (MINYELLOW, MINSAT, MINVAL), (MAXYELLOW, MAXSAT, MAXVAL))
		hsv = cv2.bitwise_and(hsv, hsv, mask = circulo_amarelo)
		
		# pega circulo azul mais interno
		circulo_azul = self.imlimiares(hsv, (MINBLUE, MINSAT, MINVAL), (MAXBLUE, MAXSAT, MAXVAL))
		hsv = cv2.bitwise_and(hsv, hsv, mask = circulo_azul)
		
		# pega cruz amarela
		#cruz_amarela = self.imlimiares(hsv, (MINYELLOW, MINSAT, MINVAL), (MAXYELLOW, MAXSAT, MAXVAL))
		#hsv = cv2.bitwise_and(hsv, hsv, mask = cruz_amarela)
		
		# image final
		self.blue = circulo_azul.copy()
		self.yellow = circulo_amarelo.copy()
		#
		self.final = cv2.bitwise_and(self.blue, self.yellow, mask = None)

		cv2.imshow("FINAL", self.final)
		#self.final = self.imfill(self.final)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNEL_RESOLUTION, KERNEL_RESOLUTION))
		self.final = cv2.morphologyEx(self.final, cv2.MORPH_OPEN, kernel, iterations = 2)
	
	######################################################################
	# get reference frame
	######################################################################
	def getRefFrame(self):
		
		#######################
		# image processing
		#######################
		self.processImage()
		
		#######################
		# get visual information
		#######################
		# para cada contorno (marcador)
		_, contours, hier = cv2.findContours(self.final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		# sucesso do calculo do frame
		success = False
		
		# para todos os contornos encontrados
		for cnt in contours:
			# get convex hull
			hull = cv2.convexHull(cnt)
			#cv2.drawContours(self.img, [hull], -1, (0, 0, 255), 3)
			
			try:
				# Fit rectangle or ellipse
				if len(hull) > 4:
					ellipse = cv2.fitEllipse(hull)
				
				# elimina elipses muito pequenas (10%)
				(xc, yc), (MA, ma), theta = ellipse
				if 2.0*max(MA, ma) < .1*max(self.size):
					continue
			except:
				#print "Erro ao detectar shape"
				continue
			
			# calcula projecao: pega pontos da imagem
			#self.image2Drect(rotrect)
			self.image2Dellipse(ellipse)
				
			# plota os pontos
			for p in self.image_points:
				cv2.circle(self.img, (int(p[0]), int(p[1])), 3, (0, 255, 255), -1)
					
			try:
				# calcula projecao
				(success, self.rot_vec, self.trans_vec) = cv2.solvePnP(self.model_points, self.image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
				#print "Rotation Vector:\n {0}".format(rot_vec)
				#print "Translation Vector:\n {0}".format(trans_vec)
			
				# projetando os eixos (25cm)
				if success:
					axis_len = 250.0
					points = np.float32([[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len], [0, 0, 0]]).reshape(-1, 3)
					axisPoints, _ = cv2.projectPoints(points, self.rot_vec, self.trans_vec, self.camera_matrix, self.dist_coeffs)
					self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
					self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
					self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
			except:
				#print "Erro calculando projecao"
				None
			
		'''# filtrando posicao
		(x, y, z) = self.trans_vec
		ALFA = .7
		self.x = ALFA*self.x + (1.0-ALFA)*x
		self.y = ALFA*self.y + (1.0-ALFA)*y
		self.z = ALFA*self.z + (1.0-ALFA)*z
		self.trans_vec = (self.x, self.y, self.z)'''
		
		return self.rot_vec, self.trans_vec, success
	
	######################################################################
	# 3D model points. RECTANGLE
	######################################################################
	def model3DRect(self):
		self.model_points = list([])
		self.model_points.append(list([0.0, 0.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, -ARESTA/2.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, 0.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([0.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, 0.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, -ARESTA/2.0, 0.0]))
		self.model_points.append(list([0.0, -ARESTA/2.0, 0.0]))
		
		# retorna np_array
		self.model_points = np.array(self.model_points, dtype=np.float32)
		
	######################################################################
	# 3D model points. ELLIPSE
	######################################################################
	def model3DEllipse(self):
		# circulo em 3D
		self.model_points = list([])
	
		# calcula a ellipse em 3D (um circulo na verdade)
		angle = np.linspace(0, 2*np.pi, RESOLUTION)
		for i in angle:
			x = RAIO*np.cos(i)
			y = RAIO*np.sin(i)
			# novo ponto do circulo
			self.model_points.append(list([x, y, 0.0]))
	
		# retorna np_array
		self.model_points = np.array(self.model_points, dtype=np.float32)
		
	######################################################################
	# 2D model points. RECTANGLE
	######################################################################
	def image2Drect(self, rect):
	
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		
		x = list([])
		y = list([])
		for p in box:
			x.append(int(p[0]))
			y.append(int(p[1]))

		# Center of rectangle in source image
		center = (np.mean(x), np.mean(y))
		
		# lista de pontos da ellipse em 2d
		self.image_points = list([])
	
		# novo ponto do circulo		
		self.image_points.append(list([center[0], center[1]]))
		self.image_points.append(list([x[0], y[0]]))
		self.image_points.append(list([(x[0]+x[1])/2, (y[0]+y[1])/2]))
		self.image_points.append(list([x[1], y[1]]))
		self.image_points.append(list([(x[1]+x[2])/2, (y[1]+y[2])/2]))
		self.image_points.append(list([x[2], y[2]]))
		self.image_points.append(list([(x[2]+x[3])/2, (y[2]+y[3])/2]))
		self.image_points.append(list([x[3], y[3]]))
		self.image_points.append(list([(x[3]+x[0])/2, (y[3]+y[0])/2]))

		# retorna np_array
		self.image_points = np.array(self.image_points, dtype=np.float32)
		
	######################################################################
	# 2D model points. ELLIPSE
	######################################################################
	def image2Dellipse(self, ellipse):

		# parametros da ellipse
		(xc, yc), (MA, ma), theta = ellipse
		a = float(ma/2.0)
		b = float(MA/2.0)
		theta = np.deg2rad(90 - theta)
	
		# lista de pontos da ellipse em 2d
		self.image_points = list([])
	
		# calcula a ellipse rotacionada em 2D
		angle = np.linspace(0, 2*np.pi, RESOLUTION)
		for i in angle:
			x = a*np.cos(i)
			y = b*np.sin(i)
			new_x = x*np.cos(theta)  + y*np.sin(theta)
			new_y = -x*np.sin(theta) + y*np.cos(theta)
			# novo ponto do circulo
			self.image_points.append(list([xc + new_x, yc + new_y]))

		# retorna np_array
		self.image_points = np.array(self.image_points, dtype=np.float32)
		
	######################################################################
	# fill holes on images
	######################################################################
	def imfill(self, img):
	
		# kernel de convolucao
		kernel = np.ones((KERNEL_RESOLUTION, KERNEL_RESOLUTION), np.uint8)
	
		# imclose
		img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=3)
	
		# imfill
		_, contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			hull = cv2.convexHull(cnt)
			cv2.drawContours(img, [hull], 0, 255, -1)
				
		return img
		
	######################################################################
	# limiares de cores
	######################################################################
	def imlimiares(self, hsv, hsvMin, hsvMax):
	
		# limiares
		hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
	
		# imfill
		hsvtresh = self.imfill(hsvtresh)
	
		return hsvtresh
	
	#########################################################################
	# show images
	#########################################################################
	def show(self):
	
		m = 1.2
		#cv2.moveWindow('RGB', 1, 1)
		cv2.imshow('RGB', self.img)
		#
		#cv2.moveWindow('Blue', int(m*self.size[1]), 1)
		#cv2.imshow('Blue', self.blue)
		#
		#cv2.moveWindow('Yellow', 1, int(m*self.size[0]))
		#cv2.imshow('Yellow', self.yellow)
		#
		#cv2.moveWindow('Final', int(m*self.size[1]), int(m*self.size[0]))
		#cv2.imshow('Final', self.final)
	
	#########################################################################
	# destrutor
	#########################################################################
	def __del__(self):
		cv2.destroyAllWindows()
