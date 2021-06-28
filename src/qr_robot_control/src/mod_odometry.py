#!/usr/bin/env python

""" CONTROL DE UN ROBOT MOVIL UTILIZANDO LOCALIZACION POR QR

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

Cx = 0.0
Cy = 0.0
Ct = 0.0

last_odom = Pose()

class pose: 
	""" 
	Representation of the pose of the robot
	"""

	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta

odom_pub = rospy.Publisher('/odom_mod', Pose, queue_size=10)

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


def callback_odom(msg):
	""" Callback executed everytime a pose is published in the topic /odom
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""

	global last_odom

	# Convert the orientation received to eulers angles
	euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

	# Define the pose of the robot from the message received
	last_odom.position.x = msg.pose.pose.position.x
	last_odom.position.y = msg.pose.pose.position.y
	last_odom.orientation.z = euler[2]

#	odom_mod = Pose()
#	odom_mod.position.x = last_odom.x + Cx
#	odom_mod.position.y = last_odom.y + Cy
#	odom_mod.position.x = last_odom.theta + Ct
#	#publish new odom
	odom_pub.publish(last_odom)

def get_last_odom():
	global last_odom
	
	return last_odom

def callback_location(msg):
	"""Callback executed everytime a pose is published in the topic /qr_location
	
	Parameters
	----------
	msg: 
	  pose of the robot in the current time.
	"""
	global last_odom
#	global Cx
#	global Cy
#	global Ct
#
#	#get last odom
#	odom = get_last_odom()
#	print(odom.theta)
#	print(msg.orientation.z)
#	#get the difference
#	Cx = msg.position.x - odom.x
#	Cy = msg.position.y - odom.y
#	Ct = msg.orientation.z - odom.theta
#	print("Errores: {}, {}, {}".format(Cx, Cy, Ct))
#	#check the odom
#	odom_mod = Pose()
#	odom_mod.position.x = odom.x + Cx
#	odom_mod.position.y = odom.y + Cy
#	odom_mod.orientation.z = odom.theta + Ct
#	print("Odometria modificada: {}, {}, {}".format(odom_mod.position.x, odom_mod.position.y, odom_mod.orientation.z))
#	print("------------------------------------")
#	#publish new odom
#	odom_pub.publish(odom_mod)

	odom_pub.publish(last_odom)

def main():
	rospy.init_node('mod_odom', anonymous=True)
	rospy.Subscriber("/odom", Odometry, callback_odom)
	rospy.Subscriber("/qr_location", Pose, callback_location)

	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
