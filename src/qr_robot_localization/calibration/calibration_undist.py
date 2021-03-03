""" CALIBRACION DE UNA WEBCAM

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""

import cv2
import numpy as np
import glob
import yaml

square_size = 23
pattern_size = (9,6)

def loadImages(): 
	"""Load calibration images.
	
	Return value
	------------
	images : 
		vector that contains the calibration images.
	
	"""

	# Define path and filenames
	folder = "/home/carmenballester/Escritorio/QR/calibration/images_undist/*.jpg"
	filenames = glob.glob(folder)
	
	images = []
	
	# Read the images and add them to the vector
	for filename in filenames: 
		image = cv2.imread(filename)
		#image_grey = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		images.append(image) 
	
	return images

def corners(images):
	""" Get the coordinates of board's corners in every image.
	
	Parameters
	----------
	images : 
		vector that contains the calibration images.
	
	Return value
	------------
	corners : 
		vector that contains for each image the coordinates of detected board's corners.

	"""

	# Define the flags and parameters
	corners = []
	cb_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
	tc_flags = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.6)

	# Search the pattern in every image of the dataset	
	for image in images:
		corners_image = cv2.findChessboardCorners(image, pattern_size,cb_flags)
		# If the pattern has beem found, refine the result
		if corners_image[0] == True:
			image_grey = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
			cv2.cornerSubPix(image_grey,corners_image[1],(10,10),(-1,-1),tc_flags)
			# Add the corners to the vector
			corners.append(corners_image[1])
		
		# Draw the results
		cv2.drawChessboardCorners(image,pattern_size,corners_image[1], corners_image[0])
		cv2.imshow('img',image)
		cv2.waitKey()

	return corners

def points(images):
	"""Define the 3D coordinates of each point in the board for every image.

	Parameters
	----------
	images : 
		vector that contains the calibration images.	

	Return value
	------------
		points : vector that contains for each image the 3D coordinates for each board's point.				

	"""

	# Define parameters
	points = []
	
	# Define the vector that contains the 3D coordinates of each point for one image. 
	points_image = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
	points_image[:,:2] = np.mgrid[0:pattern_size[0]*square_size:square_size,0:pattern_size[1]*square_size:square_size].T.reshape(-1,2)

	# Add the vetor of points to the vector of each image
	for i in range(len(images)):
		points.append(points_image)

	return points

def save(mat,coeff_dist):
	"""Save the calibration parameters.

	Parameters
	----------
	mat: 
		matrix that contais the intrinsic parameters of the camera.
	coeff_dist: 
		vector that contains the distorsion coefficients of the camera. 
	"""
	
	f = open("calibration_undist.txt", "w")
	f.write("Matrix: ")
	f.write("[{:.4f} {:.4f} {:.4f}]\n".format(mat[0][0],mat[0][1],mat[0][2]))
	f.write("\t[{:.4f} {:.4f} {:.4f}]\n".format(mat[1][0],mat[1][1],mat[1][2]))
	f.write("\t[{:.4f} {:.4f} {:.4f}]\n".format(mat[2][0],mat[2][1],mat[2][2]))
	
	f.write("Dist. Coeffs: ")
	f.write("[{:.8f} {:.8f} {:.8f} {:.8f} {:.8f}]\n".format(coeff_dist[0][0],coeff_dist[0][1],coeff_dist[0][2],coeff_dist[0][3],coeff_dist[0][4]))

	f.close()	

def main():
	""" Main proccess for calibrating the webcam using the Chess Board calibration method.

	"""

	# Load images
	images = loadImages()
	# Get 2D point's coordinates for each image
	image_points = np.array(corners(images))
	# Get 3D point's coordinates for each image (they are all the same because the board is the same, but we need that both 2D and 3D vectors have the same lenght)
	real_points = np.array(points(images))

	# Define parameters
	image_size = images[0].shape[:2]
	cam_mat = []
	dist_coeffs = []

	# If the detection has been successfull calibrate the camera
	if(len(image_points) == len(real_points)):
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(real_points,image_points,image_size,None,None)

	# Calculate the mean reprojection error of the calibration
	mean_error = 0
	for i in xrange(len(real_points)):
	    image_points_rp, _= cv2.projectPoints(real_points[i], rvecs[i], tvecs[i], mtx, dist)
	    error = cv2.norm(image_points[i], image_points_rp, cv2.NORM_L2)/len(image_points_rp)
	    mean_error += error
	print("Reprojection error: {}".format(mean_error/len(real_points)))

	save(mtx,dist)

if __name__ == "__main__": 
	main()
