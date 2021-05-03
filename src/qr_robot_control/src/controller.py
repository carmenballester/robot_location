#!/usr/bin/env python
from sys import exit

import numpy as np
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from qr_robot_control.srv import *
from qr_robot_localization.srv import *

class coefs:
	def __init__(self, A, B, C): 
		self.A = A
		self.B = B
		self.C = C

class point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

class vector:
	def __init__(self, first, last):
		self.x = last.x - first.x
		self.y = last.y - first.y

class pose: 
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta


# Create a publisher
vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10) #gazebo
# vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10) #real

# Define dict
graph = {'O':[0.5,0.0],
		 'A':[0.5, 2.0],
		 'B':[-0.5, 3.5],
		 'C':[-1.0, -3.0],
		 'D':[2.0, 0.0]} 

# Define variables
goal_reached = False

coefs_traj = coefs(0.0, 0.0, 0.0)
theta_traj = 0.0
init = point(0.0, 0.0)
goal = point(0.0, 0.0)
traj = vector(init, goal)

kd = 1
kalp = 1
kv = 1
v_max = 0.25

def mag(x):
	return math.sqrt(sum(i**2 for i in x))


def euler_from_quaternion(x, y, z, w):
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)
	
	return roll_x, pitch_y, yaw_z


def callback_odom(msg):
	global theta_traj
	global coefs_traj
	global goal
	global goal_reached
	vel_msg = Twist()

	if coefs_traj.A != 0 or coefs_traj.B != 0 or coefs_traj.C != 0 and not goal_reached:
		euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	
		pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])
	
		#Calcular ed y ealpha
		ed = ((coefs_traj.A*pos_rob.x + coefs_traj.B*pos_rob.y + coefs_traj.C)/(mag([coefs_traj.A, coefs_traj.B])))
		ealp = pos_rob.theta - theta_traj
		em = abs(mag([pos_rob.x, pos_rob.y]) - mag([goal.x, goal.y]))
	
		print("theta_rob: {:,.2f}\t theta traj: {:,.2f}".format(pos_rob.theta*180/math.pi, theta_traj*180/math.pi))
	#	print("coefs: {}, {}, {}".format(coefs_traj.A, coefs_traj.B, coefs_traj.C))
		print("Error distancia: {:,.2f}\t Error angulo: {:,.2f}\t Error magnitud: {:,.2f}".format(ed, ealp*180/math.pi, em))
	
		u = -kalp*ealp - kd*ed
		v = v_max*em - abs(u*em)*kv
	
		if abs(v) > v_max: 
			v = v_max
		if v<= 0:
			v = 0.0
		
		print("") 
		print("Stearing: {:,.2f}\t Velocity:{:,.2f}".format(u,v))
		print("-------------------------")
	
		vel_msg.linear.x = v
		vel_msg.angular.z = u
		
		if em < 0.3:
			goal_reached = True
	
		vel_pub.publish(vel_msg)


def dijkstra_client():
	rospy.wait_for_service('dijkstra')
	try: 
		dijkstra = rospy.ServiceProxy('dijkstra', Dijkstra)
		resp = dijkstra()
		return resp.node

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)


def location_client():
	rospy.wait_for_service('location')
	try: 
		location = rospy.ServiceProxy('location', Location)
		resp = location()
		return resp.node

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)


def main():
	global graph
	global goal_reached
	global traj
	global theta_traj
	global coefs_traj
	global init
	global goal
	node_goal = ""
	node_act = ""
	first = True

	# Starts a new node
	rospy.init_node('controller', anonymous=True)

	while(not rospy.is_shutdown()):
		if goal_reached or node_goal==node_act:
			print("Requesting new goal...")
			node_goal = dijkstra_client()
			if node_goal == "-1":
				print("Global goal reached. End of the process.")
				sys.exit()

			else:
				print("New goal received: {}".format(node_goal))

			print("Requesting actual position...")
			node_act = location_client()
			if node_act == "-1":
				print("Localization information not available.")
			else:
				print("Localization information received: {}".format(node_act))
	
#		if node_goal!=node_act and node_goal!="-1" and node_act!="-1":
			# Definir los puntos
			init = point(graph[node_act][0], graph[node_act][1])
			goal = point(graph[node_goal][0], graph[node_goal][1])
		
			# Calcular los parametros de la recta
			# Vector director
			traj = vector(init, goal)
			# Coeficientes de la ecuacion general: A B C
			coefs_traj.A = init.y - goal.y
			coefs_traj.B = goal.x - init.x
			coefs_traj.C = (init.x-goal.x)*init.y + (goal.y-init.y)*init.x
			# Calcular el angulo
			axis = vector(point(0.0, 0.0), point(10.0, 0.0))
			axis_np = np.array([axis.x, axis.y])
			traj_np = np.array([traj.x, traj.y])
			dot = axis.x*traj.x + axis.y*traj.y
			det = axis.x*traj.y - axis.y*traj.x
			theta_traj = math.atan2(det,dot)
			
			goal_reached = False

		if first: 	
			rospy.Subscriber("/odom", Odometry, callback_odom)
			first = False

#		rospy.spin()


if __name__ == '__main__':
	try:
		#Testing our function
		main()
	except rospy.ROSInterruptException: pass



