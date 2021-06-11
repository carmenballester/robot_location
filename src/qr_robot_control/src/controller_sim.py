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
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
# Personalized services
from qr_robot_control.srv import *
from qr_robot_localization.srv import *


# Define classes
class coefs:
	""" 
	Representation of the coefficients of a line
	"""

	def __init__(self, A, B, C): 
		self.A = A
		self.B = B
		self.C = C

class point:
	""" 
	Representation of the coordinates of a point
	"""

	def __init__(self, x, y):
		self.x = x
		self.y = y

class vector:
	""" 
	Representation of the coordinates of a vector
	"""

	def __init__(self, first, last):
		self.x = last.x - first.x
		self.y = last.y - first.y

class pose: 
	""" 
	Representation of the pose of the robot
	"""

	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta


# Create the publisher for the velocity
vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10) #gazebo
# vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10) #real

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
goal_reached = False
using_qr_location = False

# Define trajectory variables
coefs_traj = coefs(0.0, 0.0, 0.0)
theta_traj = 0.0
init = point(0.0, 0.0)
goal = point(0.0, 0.0)
traj = vector(init, goal)

# Define controller coefficients and parameters
kd = 1
kalp = 0.5
kv = 1
v_max = 0.25

# Define representation variables
odom_data = [[],[]]
qr_data = [[],[]]
gazebo_data = [[],[]]
trajectory_data = [[],[]]

def mag(x):
	"""Calculate the magnitude of a given vector.
	
	Parameters
	----------
	x: 
	  vector which magnitude is being calculated.
	"""

	return math.sqrt(sum(i**2 for i in x))


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

def callback_location(msg):
	"""Callback executed everytime a pose is published in the topic /qr_location
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	# Define global variables that are used
	global theta_traj
	global coefs_traj
	global goal
	global goal_reached
	global using_qr_location
	global qr_data

	# Check if there is some trajectory calculated and the goal is not reached
	if coefs_traj.A != 0 or coefs_traj.B != 0 or coefs_traj.C != 0 and not goal_reached:
		using_qr_location = True
	
		# Define the pose of the robot from the message received
		pos_rob = pose(msg.position.x, msg.position.y, msg.orientation.z)
	
		# ERROR between the pose and the trajectory
		# 1. Distance error - perpendicular to trajectory
		ed = ((coefs_traj.A*pos_rob.x + coefs_traj.B*pos_rob.y + coefs_traj.C)/(mag([coefs_traj.A, coefs_traj.B])))

		# 2. Angle error - between orientations
		ealp = pos_rob.theta - theta_traj

		# 3. Magnitude error - measured above trajectory
		em = abs(mag([pos_rob.x, pos_rob.y]) - mag([goal.x, goal.y]))
	
#		print("theta_rob: {:,.2f}\t theta traj: {:,.2f}".format(pos_rob.theta*180/math.pi, theta_traj*180/math.pi))
#		print("coefs: {}, {}, {}".format(coefs_traj.A, coefs_traj.B, coefs_traj.C))
#		print("Error distancia: {:,.2f}\t Error angulo: {:,.2f}\t Error magnitud: {:,.2f}".format(ed, ealp*180/math.pi, em))
		
		# CONTROL LAW
		# 1. Stearing (angle)
		u = -kalp*ealp - kd*ed

		# 2. Velocity
		v = v_max*em - abs(u*em)*kv
		# Saturate the values if necesary 
		v = v_max if abs(v)>v_max else v
		v = 0.0 if v<=0.0 else v
		
#		print("") 
#		print("Stearing: {:,.2f}\t Velocity:{:,.2f}".format(u,v))
#		print("-------------------------")

		# REPRESENTATION values
		qr_data[0].append(pos_rob.x)
		qr_data[1].append(pos_rob.y)
	
		# INTEGRATION in ros
		# 1. Define the message variable
		vel_msg = Twist()
	
		# 2. Complete the message with the control values and publish it
		vel_msg.linear.x = v
		vel_msg.angular.z = u
		
		vel_pub.publish(vel_msg)

		# Check if the goal has been reached
		goal_reached = True if em<0.3 else False

def callback_odom(msg):
	""" Callback executed everytime a pose is published in the topic /odom
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	# Define global variables that are used
	global theta_traj
	global coefs_traj
	global goal
	global goal_reached
	global using_qr_location
	global odom_data

	# Check if there is some trajectory calculated, the goal is not reached and the qr location is not being used
	if coefs_traj.A != 0 or coefs_traj.B != 0 or coefs_traj.C != 0 and not goal_reached and not using_qr_location:
		# Convert the orientation received to eulers angles
		euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	
		# Define the pose of the robot from the message received
		pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])
	
		# ERROR between the pose and the trajectory
		# 1. Distance error - perpendicular to trajectory
		ed = ((coefs_traj.A*pos_rob.x + coefs_traj.B*pos_rob.y + coefs_traj.C)/(mag([coefs_traj.A, coefs_traj.B])))

		# 2. Angle error - between orientations
		ealp = pos_rob.theta - theta_traj

		# 3. Magnitude error - measured above trajectory
		em = abs(mag([pos_rob.x, pos_rob.y]) - mag([goal.x, goal.y]))
	
#		print("theta_rob: {:,.2f}\t theta traj: {:,.2f}".format(pos_rob.theta*180/math.pi, theta_traj*180/math.pi))
#		print("coefs: {}, {}, {}".format(coefs_traj.A, coefs_traj.B, coefs_traj.C))
#		print("Error distancia: {:,.2f}\t Error angulo: {:,.2f}\t Error magnitud: {:,.2f}".format(ed, ealp*180/math.pi, em))
		
		# CONTROL LAW
		# 1. Stearing (angle)
		u = -kalp*ealp - kd*ed

		# 2. Velocity
		v = v_max*em - abs(u*em)*kv
		# Saturate the values if necesary 
		v = v_max if abs(v)>v_max else v
		v = 0.0 if v<=0.0 else v
		
#		print("") 
#		print("Stearing: {:,.2f}\t Velocity:{:,.2f}".format(u,v))
#		print("-------------------------")
	
		# REPRESENTATION values
		odom_data[0].append(pos_rob.x)
		odom_data[1].append(pos_rob.y)
	
		# INTEGRATION in ros
		# 1. Define the message variable
		vel_msg = Twist()
	
		# 2. Complete the message with the control values and publish it
		vel_msg.linear.x = v
		vel_msg.angular.z = u
		
		vel_pub.publish(vel_msg)
		
		# Check if the goal has been reached
		goal_reached = True if em<0.3 else False

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

def dijkstra_client():
	""" Client that request the next node of the global planning to the service dijkstra.

	"""

	# Wait until the service is operative
	rospy.wait_for_service('dijkstra')
	try: 
		# Define the service proxy for using it
		dijkstra = rospy.ServiceProxy('dijkstra', Dijkstra)
		
		# Get the value of the next node and return it to the main function
		resp = dijkstra()
		return resp.node

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)


def location_client():
	""" Client that request the id of the last qr seen the to the service location.

	"""

	# Wait until the service is operative
	rospy.wait_for_service('location')
	try: 
		# Define the service proxy for using it
		location = rospy.ServiceProxy('location', Location)
		# Get the value of the last qr and return it to the main function
		resp = location()
		return resp.node

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def trajectory_plot(qr_data, odom_data, trajectory_data, gazebo_data):
	""" Function representing the calculated trajectory and the diferent received positions.

	Parameters
	----------
	qr_data: 
	  positions of the robot from qr localization.
	odom_data:
	  positions of the robot from odometry.
	trajectory_data:
	  calculated trajectory.
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
		
		# Plot the calculated trajectory and the QR localizations	
		ax.plot(trajectory_data[0], trajectory_data[1], linewidth=3, label='Path')
		ax.plot(qr_mark[0], qr_mark[1], 'kx', markersize=15,)

		# Plot the positions data
		ax.plot(gazebo_data[0], gazebo_data[1], linewidth=2, label='Position-Gazebo')
		ax.plot(odom_data[0], odom_data[1], linewidth=2, label='Position-odom')
		ax.plot(qr_data[0], qr_data[1], '.', markersize=7, label='Position-QR')

		# Set the information
		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_title('Robot position')
		ax.legend()

#	plt.show()

def main():
	""" Main function that coordinates the control and the localization.

	"""

	# Define global variables that are used
	global graph
	global goal_reached
	global traj
	global theta_traj
	global coefs_traj
	global init
	global goal
	global qr_data
	global odom_data
	global trajectory_data
	global gazebo_data

	# Define auxiliar variables
	node_goal = ""
	node_act = ""

	# Initialize ROS params
	rospy.init_node('controller', anonymous=True)
	rospy.Subscriber("/odom", Odometry, callback_odom)
	rospy.Subscriber("/qr_location", Pose, callback_location)
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback_gazebo)

	while(not rospy.is_shutdown()):
		# Check if the goal has been reached
		if goal_reached or node_goal==node_act:
			# SERVICES 
			# Trajectory service call for new goal
			print("node goal={}		node act={}".format(node_goal, node_act))
			print("Requesting new goal...")
			node_goal = dijkstra_client()
			# Check if there is another local goal
			if node_goal == "-1":
				print("Global goal reached. End of the process.")
				trajectory_plot(qr_data, odom_data, trajectory_data, gazebo_data)
				rospy.spin()
#				sys.exit()

			else:
				print("New goal received: {}".format(node_goal))

			# Location service call for actual localization
			print("Requesting actual position...")
			node_act = location_client()
			# Check if there is position information
			if node_act == "-1":
				print("Localization information not available.")
			else:
				print("Localization information received: {}".format(node_act))

			# TRAJECTORY	
			# Define initial and final point using the dictionary
			init = point(graph[node_act][0], graph[node_act][1])
			goal = point(graph[node_goal][0], graph[node_goal][1])
		
			# Representation values
			trajectory_data[0].append(init.x)
			trajectory_data[1].append(init.y)
			trajectory_data[0].append(goal.x)
			trajectory_data[1].append(goal.y)
	
			# Calculate the line params
			# 1. Vector
			traj = vector(init, goal)

			# 2. Coefficients: A B C
			coefs_traj.A = init.y - goal.y
			coefs_traj.B = goal.x - init.x
			coefs_traj.C = (init.x-goal.x)*init.y + (goal.y-init.y)*init.x

			# 3. Angle with X axis
			axis = vector(point(0.0, 0.0), point(10.0, 0.0))

			dot = axis.x*traj.x + axis.y*traj.y
			det = axis.x*traj.y - axis.y*traj.x
			theta_traj = math.atan2(det,dot)

			print("")
			
			goal_reached = False

		using_qr_location = False

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
