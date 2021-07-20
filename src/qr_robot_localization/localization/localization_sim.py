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
fx = 1050.00
fy = 1050.00
dist_x = 0.23
dist_y = 0.00
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
	qr_id = "-1"	

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
			y_v = np.array([(Oi[0]-Oi[0]),(100-Oi[1])])
			x_v = np.array([(Oi[0]-100),(Oi[1]-Oi[1])])
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
			xRob = posCamG[0]+math.cos(alpha3)*mag([dist_x,dist_y])
			yRob = posCamG[1]+math.sin(alpha3)*mag([dist_x,dist_y])
			thetaRob = posCamG[2]

			posRob = np.array([xRob, yRob, thetaRob+math.pi])

			posRob[2] = posRob[2] if posRob[2]<math.pi else posRob[2]-2*math.pi

			print("-----Robot position QR {}-----------------------------------------------\n".format(index))
			print("X: {:.2f}\t\t Y: {:.2f}\t theta: {:.2f}".format(posRob[0], posRob[1], posRob[2]))
			print("-----------------------------------------------------------------------\n")
			
			# INTEGRATION in ros
			# 1. Define the message variable
			pos_msg = Pose()

			# 2. Complete the message with the robot position and publish it
			pos_msg.position.x = posRob[0]
			pos_msg.position.y = posRob[1]
			pos_msg.orientation.z = posRob[2]

			pub.publish(pos_msg)

			index += 1;

def callback_image(data): 
	""" Callback executed everytime an image is published in the topic /qr_robot/wide_camera/image_raw. 

	Parameters
	----------
	data : Image
		image captured from the camera.
	"""

	# Convert the image received in CV format
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

	# Detect QR and display results
	detectQR(cv_image)	
	cv2.imshow("Results_sin_dist",cv_image)
	cv2.waitKey(5)

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
	""" Main function where ROS parameters are initialized. 

	"""

	rospy.init_node('qr_robot_localization', anonymous=True)
	rospy.Subscriber("qr_robot/wide_camera/image_raw", Image, callback_image)
	rospy.Service('location', Location, handle_location)

	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
