"""
This file provides a wrapper to run a physics simulator. Currently only gazebo 
and bullet are supportedbut bullet should be also soon.
"""


import logging
import psutil
import rospy as ros
import subprocess
import time

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "0.1" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 14th, 2017"


class Controller():

	def __init__(self, config):

		return

	def step(self):

		return

class Sine(Controller):

	def __init__(self, config):

		super(Controller, self).__init__()

		self.n_motors = 4
		self.len = self.n_motors * 2 + 1
		self.hist_target = [[0] for i in range(self.n_motors)]
		self.hist_command = [[0] for i in range(self.n_motors)]

	def getLen(self):

		return self.len		

	def setNormalized(self, liste):

		assert len(liste) == self.len, "The size of the controller parameters is not correct"
		self.freq = liste[0] * 8
		self.amp = []
		self.phi = []
		
		j = 1
		for i in range(self.n_motors):
			self.amp.append(liste[j] * 1)
			self.phi.append(liste[j+1] * 3.1416)
			j += 2

	def step(self, t, i, pos_feed, speed_feed):

		target = self.amp[i] * math.sin(self.freq * t + self.phi[i])
		self.hist_target[i].append(target)

		return target

	def plot(self):

		plt.plot(self.hist_target[0][0:10000], "r-")
		plt.savefig("hist.png", format='png', dpi=300)
		plt.close()
 
