#!/usr/bin/env python

""" CONTROL DE UN ROBOT MOVIL UTILIZANDO LOCALIZACION POR QR

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""

from sys import exit
import numpy as np
import math
import matplotlib.pyplot as plt
import cv2

# Libraries for ROS
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
# Personalized services
from qr_robot_localization.srv import *

# Define classes
class pose: 
	""" 
	Representation of the pose of the robot
	"""

	def __init__(self, x=0.0, y=0.0, theta=0.0):
		self.x = x
		self.y = y
		self.theta = theta

# Create the publisher for the odometry
#reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

# Define the dictionary wich contains the equivalence between a qr id and its coordinates in 3D
graph = {'O':[0.5,0.0],
		 'A':[0.5, 2.0],
		 'B':[-0.5, 3.5],
		 'C':[-1.0, -3.0],
		 'D':[2.0, 0.0],
		 'E':[4.0, 1.0],
		 'F':[3.5, -1.5],
		 'G':[1.5, -2.5],
		 'H':[6.5, 0.0],
		 'I':[6.5, -3.5],
		 'J':[5.0, -3.0]} 

# Define program variables
qr_received = False
qr_detected = False
qr_added = False

# Define representation variables
odom_data = [[],[],[]]
corrected_odom_data = [[],[],[]]
qr_data = [[],[],[]]
gazebo_data = [[],[],[]]
#trajectory_data = [[],[],[]]

def euler_from_quaternion(x, y, z, w):
	"""Convert the angle given in quaternion form to euler angles.
	
	Parameters
	----------
	x: 
	  component x of the quaternion.
	y: 
	  component y of the quaternion.
	z: 
	  component z of the quaternion.
	w: 
	  component w of the quaternion.
	"""
		
	# Apply formula and get roll
	t0 = +2.0 * (w*x+y*z)
	t1 = +1.0 - 2.0 * (x*x+y*y)
	roll_x = math.atan2(t0, t1)
	
	# Apply formula and get pitch
	t2 = +2.0 * (w*y-z*x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)
	
	# Apply formula and get yaw
	t3 = +2.0 * (w*z+x*y)
	t4 = +1.0 - 2.0 * (y*y+z*z)
	yaw_z = math.atan2(t3, t4)
	
	return roll_x, pitch_y, yaw_z


def callback_qr(msg):
	"""Callback executed everytime a pose is published in the topic /qr_location
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	global qr_data
	global qr_added

	qr_added = True

	# Define the pose of the robot from the message received
	pos_rob = pose(msg.position.x, msg.position.y, msg.orientation.z)

	# Add the new data
	qr_data[0].append(pos_rob.x)
	qr_data[1].append(pos_rob.y)
	qr_data[2].append(pos_rob.theta)

#	print("QR added! Total data: {}".format(len(qr_data[0])))

	qr_added = False

def callback_odom(msg):
	""" Callback executed everytime a pose is published in the topic /odom
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	global odom_data
	global qr_detected
	global qr_added

	# COMPROBAR QUE SE ESTa LEYENDO UN CoDIGO QR
#	if qr_detected: 
#		# COMPROBAR CUANDO SE AnADE UN QR Y AnADIR TB UN ODOM
#		if qr_added:
#			# Convert the orientation received to eulers angles
#			euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
#			
#			# Define the pose of the robot from the message received
#			pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])
#		
#			# Add the new data
#			odom_data[0].append(pos_rob.x)
#			odom_data[1].append(pos_rob.y)
#			odom_data[2].append(pos_rob.theta)
#
##			print("Odom added! Total data: {}".format(len(odom_data[0])))
#			
#
#	else: 	
	# Convert the orientation received to eulers angles
	euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	
	# Define the pose of the robot from the message received
	pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

	# Add the new data
	odom_data[0].append(pos_rob.x)
	odom_data[1].append(pos_rob.y)
	odom_data[2].append(pos_rob.theta)

#		print("This should not be here :(")


def callback_gazebo(msg):
	""" Callback executed everytime a pose is published in the topic /gazebo/model_states. It is used to know the real path followed by the robot.
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	global gazebo_data
	
	# Get the mobile_base (robot) information
	for cont in range(len(msg.name)):
		i = cont if msg.name[cont]=='mobile_base' else -1

	# Add the positions to the global data
	if i != -1:
		gazebo_data[0].append(msg.pose[i].position.x)	
		gazebo_data[1].append(msg.pose[i].position.y)	
		gazebo_data[2].append(msg.pose[i].orientation.z)	


def trajectory_plot():
	""" Function representing the calculated trajectory and the diferent received positions.

	Parameters
	----------
	qr_data: 
	  positions of the robot from qr localization.
	odom_data:
	  positions of the robot from odometry.
	gazebo_data:
	  positions of the robot from Gazebo (the real positions).
	"""

	global corrected_odom_data
	global gazebo_data
	global odom_data
	global qr_data

	# Get the QR coordinates
	qr_mark = [[],[]]
	for qr in graph:
		qr_mark[0].append(graph[qr][0])
		qr_mark[1].append(graph[qr][1])

	# Create a figure and represent the data
	with plt.style.context('seaborn-pastel'):
		fig, ax = plt.subplots()
		# Set the axis
		ax.set_xlim([-2, 7])	
		ax.set_ylim([-4, 3])
		
		# Plot the calculated the QR localizations	
		ax.plot(qr_mark[0], qr_mark[1], 'kx', markersize=15,)

		# Plot the positions data
		ax.plot(gazebo_data[0], gazebo_data[1], linewidth=2, label='Position-Gazebo')
		ax.plot(odom_data[0], odom_data[1], linewidth=2, label='Position-odom')
#		ax.plot(odom_data[0], odom_data[1], '.', markersize=5, label='Position-odom')
#		ax.plot(corrected_odom_data[0], corrected_odom_data[1], linewidth=2, label='Position-corrected-odom')
		ax.plot(corrected_odom_data[0], corrected_odom_data[1], '.', markersize=7, label='Position-corrected-odom')
		ax.plot(qr_data[0], qr_data[1], '.', markersize=7, label='Position-QR')

		# Set the information
		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_title('Robot position')
		ax.legend()

	plt.savefig("okase.png")
	plt.show()
	cv2.waitKey(0)


def detecting_qr_client():
	""" Client that request the id of the qr which is being seen the to the service location.
	"""

	# Wait until the service is operative
	rospy.wait_for_service('detecting_qr')
	try: 
		# Define the service proxy for using it
		detecting_qr = rospy.ServiceProxy('detecting_qr', DetectingQR)
		# Get the value of the last qr and return it to the main function
		resp = detecting_qr()
		return resp.node

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)


def main():

	global odom_data
	global qr_data

	global corrected_odom_data
	global qr_detected

	# Initialize ROS params
	rospy.init_node('loc_controller', anonymous=True)
	rospy.Subscriber("/odom", Odometry, callback_odom)
	rospy.Subscriber("/qr_location", Pose, callback_qr)
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback_gazebo)

	loc_pub = rospy.Publisher("/robot_location", Pose, queue_size=10)
	loc_msg = Pose()

	pos_rob = pose()
	correction = pose()
	qr_received_ant = "-1"
	
	while(not rospy.is_shutdown()):
		# Location service call for actual localization
		qr_received = detecting_qr_client()
#	print(qr_received)

		# Check if there is position information
#		if qr_received == "-1" and qr_received_ant != "-1":
#			print("--------------------------------------------")
#			print("ODOM DATA: {}".format(len(odom_data[0])))
#			print("QR   DATA: {}".format(len(qr_data[0])))
#			print("--------------------------------------------")
#			print("")

		if qr_received == "-1":
			qr_detected = False

			if len(odom_data[0]) > 0:
				pos_rob.x = odom_data[0][len(odom_data[0])-1] - correction.x
				pos_rob.y = odom_data[1][len(odom_data[1])-1] - correction.y
				pos_rob.theta = odom_data[2][len(odom_data[2])-1] - correction.theta

				# Add the new data
				corrected_odom_data[0].append(pos_rob.x)
				corrected_odom_data[1].append(pos_rob.y)
				corrected_odom_data[2].append(pos_rob.theta)

		else:
			qr_detected = True
			
#			if qr_received != qr_received_ant: 
				# mientas se esta detectando QR capar la frecuencia de la odometria
#				print("--------------------------------------------")
#				print("ODOM DATA: {}".format(len(odom_data[0])))
#				print("QR   DATA: {}".format(len(qr_data[0])))
#				print("--------------------------------------------")
#				print("")


			if len(qr_data[0]) > 0 and len(odom_data[0]) > 0:
				correction.x = odom_data[0][len(odom_data[0])-1] - qr_data[0][len(qr_data[0])-1]
				correction.y = odom_data[1][len(odom_data[1])-1] - qr_data[1][len(qr_data[1])-1]
				correction.theta = odom_data[2][len(odom_data[2])-1] - qr_data[2][len(qr_data[2])-1]
 
				print("--------------------------------------------")
				print("ERR x: {}, y: {}, t: {}".format(correction.x, correction.y, correction.theta))
				print("ODOM  x: {}, y: {}, t: {}".format(odom_data[0][len(odom_data[0])-1], odom_data[1][len(odom_data[1])-1], odom_data[2][len(odom_data[2])-1]))
				print("QR    x: {}, y: {}, t: {}".format(qr_data[0][len(qr_data[0])-1], qr_data[1][len(qr_data[1])-1], qr_data[2][len(qr_data[2])-1]))
				print("--------------------------------------------")
				print("")


				pos_rob.x = qr_data[0][len(qr_data[0])-1]
				pos_rob.y = qr_data[1][len(qr_data[1])-1]
				pos_rob.theta = qr_data[2][len(qr_data[2])-1]
	
#				# Add the new data
#				corrected_odom_data[0].append(pos_rob.x)
#				corrected_odom_data[1].append(pos_rob.y)
#				corrected_odom_data[2].append(pos_rob.theta)

			qr_received_ant = qr_received



		# 3. Publicar el valor de la posicion
#		print("{:6f},{:6f},{:6f}".format(pos_rob.x, pos_rob.y, pos_rob.theta))
		loc_msg.position.x = pos_rob.x
		loc_msg.position.y = pos_rob.y
		loc_msg.orientation.z = pos_rob.theta

#		print(loc_msg)

		loc_pub.publish(loc_msg)

	trajectory_plot()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
