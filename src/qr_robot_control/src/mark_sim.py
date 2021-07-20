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

# Libraries for ROS
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

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

# Define representation variables
odom_data = [[],[],[]]
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
	global qr_received

	qr_received = True
	print(qr_received)

	# Define the pose of the robot from the message received
	pos_rob = pose(msg.position.x, msg.position.y, msg.orientation.z)

	# Add the new data
	qr_data[0].append(pos_rob.x)
	qr_data[1].append(pos_rob.y)
	qr_data[2].append(pos_rob.theta)


def callback_odom(msg):
	""" Callback executed everytime a pose is published in the topic /odom
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	global odom_data

	# Convert the orientation received to eulers angles
	euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	
	# Define the pose of the robot from the message received
	pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

	# Add the new data
	odom_data[0].append(pos_rob.x)
	odom_data[1].append(pos_rob.y)
	odom_data[2].append(pos_rob.theta)


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


def trajectory_plot(qr_data, odom_data, gazebo_data):
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
#		ax.plot(odom_data[0], odom_data[1], linewidth=2, label='Position-odom')
		ax.plot(odom_data[0], odom_data[1], '.', markersize=5, label='Position-odom')
		ax.plot(qr_data[0], qr_data[1], '.', markersize=7, label='Position-QR')

		# Set the information
		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_title('Robot position')
		ax.legend()

	plt.show()


def main():
	# Initialize ROS params
	rospy.init_node('loc_controller', anonymous=True)
	rospy.Subscriber("/odom", Odometry, callback_odom)
	rospy.Subscriber("/qr_location", Pose, callback_qr)
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback_gazebo)

	loc_pub = rospy.Publisher("/robot_location", Pose, queue_size=10)
	loc_msg = Pose()

	pos_rob = pose()
	global qr_received

	# TODO: add the main funcionality 
	
	while(not rospy.is_shutdown()):
#		print(qr_received)
		if qr_received:
			
			# 1.1. Corregir el error de la odometria actualizando el vector de error
			if len(qr_data[0]) > 0:
#				print("qr")
				pos_rob.x = qr_data[0][len(qr_data[0])-1]
				pos_rob.y = qr_data[1][len(qr_data[1])-1]
				pos_rob.theta = qr_data[2][len(qr_data[2])-1]


		else: 
		
			# 2.1. Actualizar el valor de la odometria con el vector de error

			if len(odom_data[0]) > 0:
#				print("odom")
				pos_rob.x = odom_data[0][len(odom_data[0])-1]
				pos_rob.y = odom_data[1][len(odom_data[1])-1]
				pos_rob.theta = odom_data[2][len(odom_data[2])-1]
	
		# 3. Publicar el valor de la posicion
#		print("{:6f},{:6f},{:6f}".format(pos_rob.x, pos_rob.y, pos_rob.theta))
		loc_msg.position.x = pos_rob.x
		loc_msg.position.y = pos_rob.y
		loc_msg.orientation.z = pos_rob.theta

		loc_pub.publish(loc_msg)

		qr_received = False


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
