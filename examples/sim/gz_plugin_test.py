#! /usr/bin/env python

"""
Test the gazebo plugin that controls the tigrillo robot
"""


import rospy
import math
import time

from std_msgs.msg import Float32MultiArray, MultiArrayLayout


__author__ = "Gabriel Urbain"
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be"
__status__ = "Research"
__date__ = "September 12th, 2017"


# Create node
rospy.init_node('test_tigrillo', anonymous=True)
pub = rospy.Publisher('tigrillo/legs_cmd', Float32MultiArray, queue_size=1)

# In a loop, send a actuation signal
t = 0
while True:

    val = [-math.pi/8 * math.sin(0.01*t), math.pi/8 * math.sin(0.01*t),
           math.pi/8 * math.sin(0.01*t), -math.pi/8 * math.sin(0.01*t)]

    rospy.loginfo("Publishing in tigrillo/legs_cmd topic: " + str(val))
    pub.publish(Float32MultiArray(layout=MultiArrayLayout([], 1), data=val))

    time.sleep(0.001)
    t += 1

    if t > 20000:
        exit()
