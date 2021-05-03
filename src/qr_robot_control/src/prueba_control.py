#!/usr/bin/env python
import numpy as np
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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
#vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10) #real

# Define dict
graph = {'A':[0.0,0.0],
		 'B':[0.0, 0.10],
		 'C':[0.15, 0.0],
		 'D':[0.08, 0.10],
		 'E':[0.08, 0.20]}

# Define variables
path_received = False
path = None

coefs_traj = coefs(0.0, 0.0, 0.0)
theta_traj = 0.0
init = point(0.0, 0.0)
goal = point(0.0, 0.0)
traj = vector(init, goal)

kd = 1
kalp = 1
kv = 1
v_max = 0.25

def get_path(msg):
	global path
	global path_received
	
	path_received = True
	path = msg.data


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


def callback(msg):
	global theta_traj
	global coefs_traj
	global goal
	vel_msg = Twist()
	
	euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

	pos_rob = pose(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

#	print(pos_rob.x, pos_rob.y, pos_rob.theta)

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

	vel_pub.publish(vel_msg)


def mag(x):
	return math.sqrt(sum(i**2 for i in x))


def main():
#	global graph
#	global path
#	global path_received
	global traj
	global theta_traj
	global coefs_traj
	global init
	global goal


	# Starts a new node
	rospy.init_node('controller', anonymous=True)
#	rospy.Subscriber("shortest_path", String, get_path)	
#
#	print("Waiting for path...")	
#	while(not path_received):
#		pass
#
#	print("Path received! It is: {}".format(path))

	#ENTRADA DE DATOS: PUNTO INICIAL Y PUNTO FINAL
	#DATOS QUE VIENEN DEL NODO  DE LOCALIZACION: PR
	#Suponemos que tenemos todo eso para las pruebas
	
	# Definir los puntos
	init = point(0.5,0.0)
	goal = point(-1.0,-3.0)

	# Calcular los parametros de la recta
	# Vector director
	traj = vector(init, goal)
	# Coeficientes de la ecuacion general: A B C
	coefs_traj.A = init.y - goal.y
	coefs_traj.B = goal.x - init.x
	coefs_traj.C = (init.x-goal.x)*init.y + (goal.y-init.y)*init.x
	#print("{}x + {}y + {}".format(coefs_traj.A, coefs_traj.B, coefs_traj.C))
	# Calcular el angulo
	axis = vector(point(0.0, 0.0), point(10.0, 0.0))
	axis_np = np.array([axis.x, axis.y])
	traj_np = np.array([traj.x, traj.y])


#	dot = x1*x2 + y1*y2      # dot product
#	det = x1*y2 - y1*x2      # determinant
#	angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

	dot = axis.x*traj.x + axis.y*traj.y
	det = axis.x*traj.y - axis.y*traj.x
	theta_traj = math.atan2(det,dot)

#	if theta_traj < 0:
#		theta_traj = theta_traj + math.pi*2

#	theta_traj = math.acos((np.dot(traj_np,axis_np))/(mag(traj_np)*mag(axis_np)))

	print("theta_traj main: {}".format(theta_traj*180/math.pi))

	rospy.Subscriber("/odom", Odometry, callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		#Testing our function
		main()
	except rospy.ROSInterruptException: pass



