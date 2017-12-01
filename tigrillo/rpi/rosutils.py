
"""
This file contains utlities to communicate to the NRP over ROS
"""

import logging
import rospy as ros
import time
import threading
from std_msgs.msg import Float32MultiArray, MultiArrayLayout


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "November 25th, 2017"


class TigrilloROS(threading.Thread):

	def __init__(self, node_name="tigrillo_node", act_name="tigrillo_act", sen_name="tigrillo_sen"):

		super(TigrilloROS, self).__init__()
		self.log = logging.getLogger('TigrilloROS')

		# ROS parameters
		self.node_name = node_name
		self.pub_rate = 2
		self.pub_name = sen_name
		self.sub_name = act_name
		self.queue_size = 1
		self.pub = None
		self.sub = None

		self.sensors = None
		self.sensors_t = 0
		self.actuators_t = 0
		self.actuators = {"FL": 0, "FR": 0, "BR": 0, "BL": 0}

		self.stop = True
		self.daemon = True

	def update_sensors(self, update, t):

		self.actuators_t = t
		self.sensors = update

	def get_last_actuators(self):

		return self.actuators

	def __ros_sub(self, msg):

		self.actuators = msg.data
		self.actuators_t = msg.t
		ros.loginfo("Actuation update received : " + str(self.actuators))

	def __ros_pub(self):

		rate = ros.Rate(self.pub_rate)
		while not ros.is_shutdown():
			ros.loginfo("Last sensed at time " + str(self.sensors_t) + " : " + str(self.sensors))
			self.pub.publish(Float32MultiArray(layout=MultiArrayLayout([], 1), data=self.sensors))
			rate.sleep()

		return

	def start(self):

		ros.init_node(self.node_name)
		self.pub = ros.Publisher(self.pub_name, Float32MultiArray, queue_size=self.queue_size)
		self.sub = ros.Subscriber(self.sub_name, Float32MultiArray, callback=self.__ros_sub, queue_size=self.queue_size)

		self.stop = False
		super(TigrilloROS, self).start()

	def stop(self):

		self.stop = True

	def run(self):

		while not self.stop:
			try:
				self.__ros_pub()
			except ros.ROSInterruptException:
				pass


# Run test of a ros node
if __name__ == '__main__':
	t = TigrilloROS()
	t.start()

	while 1:
		print("The node runs in background")
		time.sleep(1)