""" CORRECCION DE LA DISTORSION ANGULAR

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""
import cv2
import numpy as np
import zbar
import yaml
# from math import sqrt
import math

def displayShape(frame, corners) :
	"""Displays QR shape information in the actual frame.

	Parameters
	----------
	frame : numpy.ndarray
	  image captured from the camera.
	corners : tuple
	  vector that contains the coordinates of each QR corner in image space.

	"""

	n = len(corners)

	# Display outline
	cornersNp = np.array(corners, np.int32)
	cv2.polylines(frame, [cornersNp], True, (213,179,127), 2)
	
	# Display corners
	cv2.circle(frame, corners[0], 5, (199,153,84), -1) #Sup left
	cv2.circle(frame, corners[1], 5, (199,153,84), -1) #Inf left
	cv2.circle(frame, corners[2], 5, (199,153,84), -1) #Inf right
	cv2.circle(frame, corners[3], 5, (199,153,84), -1) #Sup right

def displayLabel(frame, imageCorners, realCorners) :
    """Displays QR label in the actual frame.

    Parameters
    ----------
    frame : numpy.ndarray
        image captured from the camera.
    imageCorners : tuple
        vector that contains the coordinates of each QR's corner in image space.
    realCorners : numpy.ndarray
        vector that contains the coordinates of each QR's corner in real space.

    """

    cv2.putText(frame,"A",(imageCorners[0][0],imageCorners[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (141,97,31), 2)
    cv2.putText(frame,"B",(imageCorners[1][0],imageCorners[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (141,97,31), 2)
    cv2.putText(frame,"C",(imageCorners[2][0],imageCorners[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (141,97,31), 2)
    cv2.putText(frame,"D",(imageCorners[3][0],imageCorners[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (141,97,31), 2)

#	# Get coordinates in string format
#	c1 = "("+str(realCorners[0][0])+","+str(realCorners[0][1])+","+str(realCorners[0][2])+")"
#	c2 = "("+str(realCorners[1][0])+","+str(realCorners[1][1])+","+str(realCorners[1][2])+")"
#	c3 = "("+str(realCorners[2][0])+","+str(realCorners[2][1])+","+str(realCorners[2][2])+")"
#	c4 = "("+str(realCorners[3][0])+","+str(realCorners[3][1])+","+str(realCorners[3][2])+")"
#	 
#	# Show coordinates in the console
#	print("3D QR coordinates: {}".format(c1+c2+c3+c4)) # Sup left, Inf left, Inf right, Sup right
#
#	# Display data attached to the QR Code
#	cv2.putText(frame,c1,(imageCorners[0][0],imageCorners[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (196,215,118), 2)
#	cv2.putText(frame,c2,(imageCorners[1][0],imageCorners[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (196,215,118), 2)
#	cv2.putText(frame,c3,(imageCorners[2][0],imageCorners[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (196,215,118), 2)
#	cv2.putText(frame,c4,(imageCorners[3][0],imageCorners[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (196,215,118), 2)
	
def getRotation(frame, corners) :
    """Gets QR rotation information.

    Parameters
    ----------
    frame : numpy.ndarray
        image captured from the camera.
    corners : tuple
        vector that contains the coordinates of each QR's corner in image space.

    """

    # Calculate angle as tangent of the slope of the line between two corners, converting radians to degrees
    vector = [corners[3][0]-corners[0][0], corners[3][1]-corners[0][1]]
    angle = (np.arctan2(float(vector[1]),vector[0])*57.29)%360
    # Correct angle due to the order of the screen coordinates
    angle += -360
    angle *= -1

    # Show angle in the console
    # print("QR Angle: {}".format(angle))

    # Display angle in the center of the QR Code
    cx = (corners[0][0]+corners[2][0])/2
    cy = (corners[0][1]+corners[2][1])/2
    cv2.putText(frame,str("%.2f" % angle),(cx,cy+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (196,215,118), 2)

def getAngles(frame, imageCorners, realCorners) :
	
	# Define coordinates
	xA = imageCorners[0][0]
	yA = imageCorners[0][1]

	xB = imageCorners[1][0]
	yB = imageCorners[1][1]

	xC = imageCorners[2][0]
	yC = imageCorners[2][1]
	
	xD = imageCorners[3][0]
	yD = imageCorners[3][1]

	# Calculate image distances
	dist_AB = math.sqrt(math.pow((xB-xA),2) + (math.pow((yB-yA),2)))
	dist_DC = math.sqrt(math.pow((xC-xD),2) + (math.pow((yC-yD),2)))
	dist_AD = math.sqrt(math.pow((xD-xA),2) + (math.pow((yD-yA),2)))
	dist_BC = math.sqrt(math.pow((xC-xB),2) + (math.pow((yC-yB),2)))

	# Calculate image angles
	ang_AB = math.atan2((yB-yA),(xB-xA))*180/math.pi
	ang_DC = math.atan2((yC-yD),(xC-xD))*180/math.pi
	ang_AD = math.atan2((yD-yA),(xD-xA))*180/math.pi
	ang_BC = math.atan2((yC-yB),(xC-xB))*180/math.pi

#	print("Distancia AB = {}".format(dist_AB))
#	print("Distancia DC = {}".format(dist_DC))
#	print("Distancia AD = {}".format(dist_AD))
#	print("Distancia BC = {}".format(dist_BC))
#
#	print("Angulo AB = {}".format(ang_AB))
#	print("Angulo DC = {}".format(ang_DC))
#	print("Angulo AD = {}".format(ang_AD))
#	print("Angulo BC = {}".format(ang_BC))
#
#	print("-----------------------------")

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

    for QRCode in imageZBar:
        # Save corners and label attached to every QR Code
        imageCorners = QRCode.location
        tag = QRCode.data

        # Inicialite values for getting the coordinates from the tag
        pri = 0
        fin = 0
        coor = [0]*12
        j = 0

        for i in range(len(tag)):
            if (tag[i] == ","):
                fin = i
                coor[j] = tag[pri:fin]
                pri = i+1
                j = j+1

        # Tag is normalized for getting real coordinates of the points
        x1 = float(coor[0])
        y1 = float(coor[1])
        z1 = float(coor[2])
        x2 = float(coor[3])
        y2 = float(coor[4])
        z2 = float(coor[5])
        x3 = float(coor[6])
        y3 = float(coor[7])
        z3 = float(coor[8])
        x4 = float(coor[9])
        y4 = float(coor[10])
        z4 = float(coor[11])

        imageCornersNp = np.array(imageCorners, np.float32)
        realCorners = np.float32([[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4]])
	
        if len(imageCorners) == 4:
            # Get corners and shape of the QR Code in the original frame
			displayShape(frame, imageCorners)
			# Get decodified data from the QR Code
			displayLabel(frame,imageCorners, realCorners)
			# Get angle of the QR Code
			# getRotation(frame, imageCorners)
			# Get distorsion angle of the QR Code
			getAngles(frame, imageCornersNp, realCorners)
	
def undist(frame):
	
	mtx = np.array([[967.6876,0.,672.6019],
		   			[0.,974.3047,403.3113],
 		   			[0.,0.,1.]])
	dist = np.array([[-0.39562631,0.14751944,-0.00078634,-0.00035870,-0.04135439]])

	h,w = frame.shape[:2]
	new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	
	image = cv2.undistort(frame, mtx, dist, None, new_mtx)
	# crop the image
	x, y, w, h = roi
	image = image[y:y+h, x:x+w]
	
	return image
	


def main(): 
	# Initialize camera
	capture = cv2.VideoCapture(1)
  	capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
  	width = 1080
  	height = 720
  	capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  	capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	while (cv2.waitKey(1) == -1):
		# Get one frame
		ok, frame = capture.read()
		c_frame = undist(frame)
    	# Check if the frame has been successfully read
		if ok:
			detectQR(c_frame)	
		
			# Display results
			cv2.imshow("Results",c_frame)

	# When a key is presed, release the capture, close the window and finish the program
	capture.release()
	cv2.destroyAllWindows()
	exit()

if __name__ == "__main__":
	main()
