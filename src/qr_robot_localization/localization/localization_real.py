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

# Libraries for ROS
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from qr_robot_localization.srv import Location, LocationResponse

# Define fx and fy from calibration 
fx = 670.00
fy = 1000.00
dist_x = 0.46
dist_y = 0.13
qr_id = "-1"	

pub = rospy.Publisher('/qr_location', Pose, queue_size=10)

def mag(x):
	"""Calculate the magnitude of a given vector.
	
	Parameters
	----------
	x : 
	  vector which magnitude is being calculed.
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
	"""Displays the Y axis of both QR and image, and the vector between the centers of the coordinate systems.
	
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
	cv2.arrowedLine(frame, (int(Oi[0]), int(Oi[1])), (int(Oi[0]-100), int(Oi[1])), (0,0,255), 2) # Y Image 
	cv2.arrowedLine(frame, (int(Oi[0]), int(Oi[1])), (int(Oi[0]), int(Oi[1])-100), (0,0,255), 2) # X Image 
	cv2.arrowedLine(frame, (int(Qi[0]), int(Qi[1])), (int(Qi[0]+q_v[0]), int(Qi[1])+q_v[1]), (235,0,255), 2) # Y QR 
	cv2.arrowedLine(frame, (int(Qi[0]), int(Qi[1])), (int(Qi[0]+p_v[0]), int(Qi[1])+p_v[1]), (235,0,255), 2) # X QR 
	cv2.putText(frame, "y", (int(Oi[0]-100+10), int(Oi[1])-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2) # Y Label Image
	cv2.putText(frame, "x", (int(Oi[0]+10), int(Oi[1])-100+10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2) #X Label Image
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
	
	# Define global variables that are used
	global fx
	global fy
	global dist
	global qr_id

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
			# Id of the QR
			qr_id = tag[1]
			# Inicialite values for getting the coordinates from the tag
			pri = 3 
			fin = 0
			coor = np.zeros(4)
			j = 0

			for i in range(3,len(tag)):
				if tag[i] == "," or tag[i] == ")":
					fin = i
					coor[j] = float(tag[pri:fin])
					pri = i+1
					j = j+1

			# Tag is normalized for getting real coordinates of the points
			Qr = np.array([coor[0], coor[1], coor[2], coor[3]]) # center
		
			# LOCATION of the camera
			# 1. Calculate L
			ax = Oi[0]-Qi[0]
			ay = Oi[1]-Qi[1]
			L = math.sqrt(ax**2 + ay**2)
			
			# 2. Calculate vectors
			q_v = np.array([(Ai[0]-Bi[0]),(Ai[1]-Bi[1])])
			p_v = np.array([(Di[0]-Ai[0]),(Di[1]-Ai[1])])
			a_v = np.array([ax,ay])
			y_v = np.array([(100-Oi[0]),(Oi[1]-Oi[1])])
			x_v = np.array([(Oi[0]-Oi[0]),(100-Oi[1])])
			Y_v = np.array([((Qi[0]+q_v[0])-Qi[0]),((Qi[1]+q_v[1])-Qi[1])])
			X_v = np.array([((Qi[0]+p_v[0])-Qi[0]),((Qi[1]+p_v[1])-Qi[1])])

			displayVectors(frame, Oi, Qi, q_v, p_v)

			# 3. Calculate alpha3 - angle between Ycam and Yqr
			dot = y_v[0]*Y_v[0] + y_v[1]*Y_v[1]
			det = y_v[0]*Y_v[1] - y_v[1]*Y_v[0]
			alpha3 = math.atan2(det,dot)

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

			# 8.Global coordinates - different axis
			rot = np.array([[math.cos(Qr[2]), math.sin(Qr[2]), 0],
				   [-math.sin(Qr[3]), math.cos(Qr[3]), 0],
				   [0, 0, 1]])
			posCam = np.array([[xCam],[yCam],[thetaCam]])
			posQr = np.array([Qr[0], Qr[1], Qr[3]])

			posCamG = np.transpose(np.dot(rot,posCam))[0] + posQr
			
			# LOCATION of the robot
			# Calculate robot position using angles between frames and geometry of the robot
			xRob = posCamG[0]-math.cos(alpha3)*dist_x
			yRob = posCamG[1]-math.cos(alpha3)*dist_y
			thetaRob = posCamG[2]

			posRob = np.array([xRob, yRob, thetaRob])

			print("-----Robot position QR {}-----------------------------------------------\n".format(index))
			print("X: {:.2f}\t\t Y: {:.2f}\t theta: {:.2f}".format(posRob[0], posRob[1], posRob[2]))
			print("-----------------------------------------------------------------------\n")
			
			# INTEGRATION in ros
			# 1. Define the message variable
			pos_msg = Pose()

			# 2. Complete the message with the robot position and publish it
			pos_msg.position.x = xRob
			pos_msg.position.y = yRob
			pos_msg.orientation.z = thetaRob

			pub.publish(pos_msg)

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

def handle_location(req):
	""" Callback executed everytime the location service is called. 

	Parameters
	----------
	req : none

	"""

	# Get the id of the last QR seen and send it
	global qr_id
	return LocationResponse(qr_id)

def main():
	""" Main function where ROS parameters and camera are initialized. 

	"""

	# Initialize ROS
	rospy.init_node('qr_robot_localization', anonymous=True)
	rospy.Service('location', Location, handle_location)
	
	# Initialize camera
	capture = cv2.VideoCapture(1)
  	capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
  	width = 1080
  	height = 720
  	capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  	capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

	while not rospy.is_shutdown():
		# Get one frame
		ok, frame = capture.read()
		c_frame = undist(frame)
    	# Check if the frame has been successfully read
		if ok:
			detectQR(c_frame)	

			# Display results
			cv2.imshow("Results_no_dist",c_frame)
			cv2.waitKey(5)

	# When a key is presed, release the capture, close the window and finish the program
	capture.release()
	cv2.destroyAllWindows()
	exit()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
