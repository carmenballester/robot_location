#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty
import time

# set up node
rospy.init_node('reset_odom')

# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

# reset odometry (these messages take a few iterations to get through)
timer = time.time()
print("Reseting odometry...")
while time.time() - timer < 0.30:
	reset_odom.publish(Empty())

print("Done! Time: {}".format(time.asctime(time.localtime())))
