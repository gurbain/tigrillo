"""
This file provides a wrapper to run a physics simulator. Currently only gazebo 
and bullet are supportedbut bullet should be also soon.
"""

import ast
import math
import pickle as pkl
import logging
import psutil
import rospy as ros
import subprocess
import sys
import time
import traceback

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

		# Retrieve the config
		self.config = config
		self.log = logging.getLogger('Controller')
		params = config["Controller"]

		if "file" in params:
			self.load(params["file"])
		else:
			if "params" in params:
				self.params = ast.literal_eval(params["params"])

			if "timestep" in params:
				self.timestep = float(params["timestep"])
			else:
				self.timestep = 0.01

			if "openloop" in params:
				self.openloop = params["openloop"]
			else:
				self.openloop = True

		self.it = 0
		self.t = 0
		self.t_init = 0
		self.st_timestep = 0

	def get_params_len(self):

		length = 0
		for m in self.params:
			length += len(m)

		return length

	def get_norm_params(self):

		# Surcharge it depending on the controller
		return

	def set_norm_params(self, liste):

		# Surcharge it depending on the controller
		return

	def get_params(self):

		return self.params
		
	def set_params(self, params):

		self.params = params

	def step(self, t):
		"""
		This function is called to update the controller state at each timestep
		"""

		# This function must be surcharged

		self.t = t

		cmd = []
		for i in range(len(self.params)):
			cmd.append(0)

		return cmd

	def run(self, simtime, physics):
		"""
		This function is a blocking function that execute the controller for a given simtime
		"""

		try:

			self.t_init = time.time()

			# Wait for the physics to be started
			while 1:
				time.sleep(self.timestep)
				if physics.is_sim_started():
					break;

			while physics.sim_duration < simtime :
				rt = time.time() - self.t_init
				st = physics.sim_duration
				self.st_timestep = self.timestep*st/rt

				cmd = self.step(st)
				physics.set_sim_cmd(cmd)
				time.sleep(self.st_timestep)
				
		except:
			self.log.error("Simulation aborted by user. Physics time: " + \
				str(physics.sim_duration) + "s. Controller time: not set!")
			physics.kill_sim()
			traceback.print_exc()
			sys.exit()

	def load(self, filename):
		"""
		Load itself from a pickle file
		"""
		f = open(filename,'rb')
		tmp_dict = pkl.load(f)
		f.close()          

		self.__dict__.update(tmp_dict)

	def save(self, filename):
		"""
		Save class and variables with pickle
		"""

		f = open(filename,'wb')
		pkl.dump(self.__dict__, f, 2)
		f.close()


class Sine(Controller):

	def __init__(self, config):

		super(Sine, self).__init__(config)

		self.n_motors = 4
		self.hist_cmd = [[0] for i in range(self.n_motors)]

		self.norm_f = 2 * math.pi * 10
		self.norm_a = 3
		self.norm_phi = 2 * math.pi

	def set_norm_params(self, liste):

		j = 0
		for i in range(self.n_motors):
			f = liste[j] / self.norm_f 
			a = liste[j+1] / self.norm_a
			phi = liste[j+2] / self.norm_phi
			self.params.append({"f": f, "a": a, "phi": phi})
			j += 3

	def get_norm_params(self):

		liste = []
		for m in self.params:
			for i in m:
				liste.append(m)

		return liste

	def step(self, t):

		self.t = t
		cmd = []
		for i in range(self.n_motors):
			cmd.append(self.params[i]["a"] * math.sin( \
				self.params[i]["f"] * self.t + self.params[i]["a"]))
		
		self.hist_cmd.append(cmd)
		return cmd

	def plot(self, filename="history.png"):

		plt.plot(self.hist_cmd[0][0:10000], "r-")
		plt.savefig(filename, format='png', dpi=300)
		plt.close()
 
