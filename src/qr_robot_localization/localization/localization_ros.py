#!/usr/bin/env python

""" LOCALIZACION DE UN ROBOT MOVIL UTILIZANDO QR

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""
import cv2
import numpy as np
import zbar
import yaml
import math

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Define fx and fy from calibration 
fx = 1050.00
fy = 1050.00


def mag(x):
	"""Calculate the magnitude of a given vector
	
	Parameters
	----------
	x : 
	  vector which magnitude is being calculed
	""" 

	return math.sqrt(sum(i**2 for i in x))

def displayPoints(frame, Oi, Qi) :
	"""Displays the center of the QR and the image in the actual frame.

	Parameters
	----------
	frame : numpy.ndarray
	  image captured from the camera.
	Oi : 
	  Coordinates of the image's center.
	Qi : 
	  Coordinates of the QR's center.
	"""

	# Display points
	cv2.circle(frame, (int(Oi[0]), int(Oi[1])), 5, (0,0,255), -1) # Center image
	cv2.circle(frame, (int(Qi[0]), int(Qi[1])), 5, (235,0,255), -1) # Center QR


def displayShape(frame, corners):
	"""Displays the outline of the QR.

	Parameters
	----------
	frame : numpy.ndarray
	  image captured from the camera.
	corners : 
	  image coordinates of the QR's corners.
	""" 

	cornersNp = np.array(corners, np.int32)
		
	if len(cornersNp) == 4:
		cv2.polylines(frame,[cornersNp],True,(235,0,255),2)

def displayVectors(frame, Oi, Qi, q_v, p_v):
	"""Displays the Y axis of both QR and image, and the vector between the centers of the coordinate systems
	
	Parameters
	----------
	frame : numpy.ndarray
	  image captured from the camera.	
	Oi : 
	  Coordinates of the image's center.
	Qi : 
	  Coordinates of the QR's center.
	q_v : 
	  vector between the origin of both coordinate systems. 
	"""

	# Display axis
	cv2.arrowedLine(frame, (int(Oi[0]), int(Oi[1])), (int(Oi[0]), int(Oi[1])-100), (0,0,255), 2) # Y Image 
	cv2.arrowedLine(frame, (int(Oi[0]), int(Oi[1])), (int(Oi[0]+100), int(Oi[1])), (0,0,255), 2) # X Image 
	cv2.arrowedLine(frame, (int(Qi[0]), int(Qi[1])), (int(Qi[0]+q_v[0]), int(Qi[1])+q_v[1]), (235,0,255), 2) # Y QR 
	cv2.arrowedLine(frame, (int(Qi[0]), int(Qi[1])), (int(Qi[0]+p_v[0]), int(Qi[1])+p_v[1]), (235,0,255), 2) # X QR 
	cv2.putText(frame, "y", (int(Oi[0]+10), int(Oi[1])-100+10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2) # Y Label Image
	cv2.putText(frame, "x", (int(Oi[0]+100+10), int(Oi[1])+10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2) #X Label Image
	cv2.putText(frame, "Y", (int(Qi[0]+q_v[0]), int(Qi[1])+q_v[1]-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (235,0,255), 2) # Y Label QR
	cv2.putText(frame, "X", (int(Qi[0]+p_v[0]), int(Qi[1])+p_v[1]-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (235,0,255), 2) # X Label QR

	# Display vector a	
	cv2.arrowedLine(frame, (int(Qi[0]), int(Qi[1])), (int(Oi[0]), int(Oi[1])), (255,35,0), 2)
	cv2.putText(frame, "a", (int(Qi[0]+10), int(Qi[1])-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,35,0), 2) # Label

def detectQR(frame):
	""" Detects QR Codes with ZBar.

	Parameters
	----------
	frame : numpy.ndarray
		image captured from the camera.
    """

	# Convert frame to BW and get its size
	frameGrey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # array with height, widht and channels
	size = frameGrey.shape

	# Convert image from OpenCV to ZBar
	imageZBar = zbar.Image(size[1], size[0], 'Y800', frameGrey.tobytes())

	# Construct an scan object and detect all QRCodes at frame
	scan = zbar.ImageScanner()
	scan.scan(imageZBar)
	
	index = 1

	for QRCode in imageZBar:
		# Save corners and label attached to every QR Code
		imageCorners = QRCode.location
		tag = QRCode.data
    
		if len(imageCorners) == 4: 	
			# IMAGE VALUES
			Qi_x = (float(imageCorners[0][0]+imageCorners[1][0]+imageCorners[2][0]+imageCorners[3][0]))/4.0	
			Qi_y = (float(imageCorners[0][1]+imageCorners[1][1]+imageCorners[2][1]+imageCorners[3][1]))/4.0	
			Oi = np.array([frame.shape[1]/2, frame.shape[0]/2])
			Qi = np.array([Qi_x, Qi_y]) # center
			Ai = np.array([imageCorners[0][0], imageCorners[0][1]])
			Bi = np.array([imageCorners[1][0], imageCorners[1][1]])
			Di = np.array([imageCorners[3][0], imageCorners[3][1]])
			
			displayPoints(frame,Oi,Qi)
			displayShape(frame,imageCorners)	

			# REAL VALUES
			# Inicialite values for getting the coordinates from the tag
			pri = 1
			fin = 0
			coor = np.zeros(2)
			j = 0

			for i in range(len(tag)):
				if tag[i] == "," or tag[i] == ")":
					fin = i
					coor[j] = float(tag[pri:fin])
					pri = i+1
					j = j+1

			# Tag is normalized for getting real coordinates of the points
			Qr = np.array([coor[0]*0.01, coor[1]*0.01]) # center

			# LOCATION of the camera
			# 1. Calculate L
			ax = Oi[0]-Qi[0]
			ay = Oi[1]-Qi[1]
			L = math.sqrt(ax**2 + ay**2)
			
			# 2. Calculate vectors
			q_v = np.array([(Ai[0]-Bi[0]),(Ai[1]-Bi[1])])
			p_v = np.array([(Di[0]-Ai[0]),(Di[1]-Ai[1])])
			a_v = np.array([ax,ay])
			y_v = np.array([(Oi[0]-Oi[0]),(100-Oi[1])])
			x_v = np.array([(Oi[0]-100),(Oi[1]-Oi[1])])
			Y_v = np.array([((Qi[0]+q_v[0])-Qi[0]),((Qi[1]+q_v[1])-Qi[1])])
			X_v = np.array([((Qi[0]+p_v[0])-Qi[0]),((Qi[1]+p_v[1])-Qi[1])])

			displayVectors(frame, Oi, Qi, q_v, p_v)

			# 3. Calculate alpha3 - angle between Ycam and Yqr
			alpha3 = math.acos((np.dot(y_v, Y_v))/(mag(y_v)*mag(Y_v)))

			# 4. Calculate alpha1
			alpha1 = math.acos((np.dot(a_v, Y_v))/(mag(a_v)*mag(Y_v)))

			# 5. Calculate alpha2
			alpha2 = math.acos((np.dot(a_v, X_v))/(mag(a_v)*mag(X_v)))

			# 6. Calculate unit
			unitx = 1/fx 
			unity = 1/fy

			# 7. Relative coordinates
			xCam = L*unitx*math.cos(alpha2) 
			yCam = L*unity*math.cos(alpha1)
			thetaCam = alpha3

			# 8. Global coordinates - same axis
#			xRob = xCam + Qr[0]*0.01 
#			yRob = yCam + Qr[1]*0.01
#			thetaRob = thetaCam
#
			# 8.Global coordinates - different axis
			beta_x = 0 #coor[2]
			beta_y = 0 #coor[3]
			Qr = np.append(Qr, beta_x)
			Qr = np.append(Qr, beta_y)

			rot = np.array([[math.cos(Qr[2]), math.sin(Qr[2]), 0],
				   [-math.sin(Qr[3]), math.cos(Qr[3]), 0],
				   [0, 0, 1]])
			posCam = np.array([[xCam],[yCam],[thetaCam]])
			posQr = np.array([Qr[0], Qr[1], Qr[3]])

			posCamG = np.transpose(np.dot(rot,posCam))[0] + posQr
			
			# LOCATION of the robot
			# 1. Calculate alpha4 - angle between Ycam and Xqr
			alpha4 = math.acos((np.dot(Y_v, x_v))/(mag(Y_v)*mag(x_v)))

			# 2. Calculate robot position using angles between frames and geometry of the robot
			dist = 0.23			

			xRob = posCamG[0]+math.cos(alpha3)*dist
			yRob = posCamG[1]+math.cos(alpha4)*dist
			thetaRob = posCamG[2]

			posRob = np.array([xRob, yRob, thetaRob])

			print("-----Robot position QR {}-----------------------------------------------\n".format(index))
			print("X: {:.2f}\t\t Y: {:.2f}\t theta: {:.2f}".format(posRob[0], posRob[1], posRob[2]*180/math.pi))
			print("-----------------------------------------------------------------------\n")

			index += 1;

def undist(frame):
	""" Correct the angular distorsion of the camera. 

	Parameters
	----------
	frame : numpy.ndarray
		image captured from the camera.
	"""
	
	# Define intrinsic parameters from calibration
	mtx = np.array([[967.6876,0.,672.6019],
		   			[0.,974.3047,403.3113],
 		   			[0.,0.,1.]])
	dist = np.array([[-0.39562631,0.14751944,-0.00078634,-0.00035870,-0.04135439]])

	# Undistort the image
	h,w = frame.shape[:2]
	new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	image = cv2.undistort(frame, mtx, dist, None, new_mtx)
	
	# Crop the image
	x, y, w, h = roi
	image = image[y:y+h, x:x+w]

	return image

def callback(data): 
	# Get one frame
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

	detectQR(cv_image)	
	# Display results
	cv2.imshow("Results_sin_dist",cv_image)
	cv2.waitKey(5)

def listener():
	rospy.init_node('qr_robot_localization', anonymous=True)
	rospy.Subscriber("qr_robot/wide_camera/image_raw", Image, callback)

	rospy.spin()

if __name__ == "__main__":
	listener()
