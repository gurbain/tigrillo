#! /usr/bin/env python

import rospy
import math
import time

from std_msgs.msg import Float32MultiArray, MultiArrayLayout

# Create node
rospy.init_node('test_tigrillo', anonymous=True)
pub = rospy.Publisher('tigrillo/legs_cmd', Float32MultiArray, queue_size=1)

# In a loop, send a actuation signal
t = 0
while True:
	
	val = [-math.pi/8 * math.sin(0.01*t), math.pi/8 * math.sin(0.01*t), \
		math.pi/8 * math.sin(0.01*t), -math.pi/8 * math.sin(0.01*t)]

	rospy.loginfo("Publishing in tigrillo/legs_cmd topic: " + str(val))
	pub.publish(Float32MultiArray(layout=MultiArrayLayout([], 1), data=val))
	
	time.sleep(0.001)
	t += 1
	
	if t > 20000:
		exit()