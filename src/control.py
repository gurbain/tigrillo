"""
This file provides a wrapper to run a physics simulator. Currently only gazebo 
and bullet are supportedbut bullet should be also soon.
"""

import ast
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import logging
import pickle as pkl
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
			else:
				self.params = dict()

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

		self.hist_cmd = []

	def __getstate__(self):
		""" This is called before pickling. """

		state = self.__dict__.copy()
		del state['hist_cmd']
		del state["log"]

		return state

	def __setstate__(self, state):
		""" This is called while unpickling. """

		self.__dict__.update(state)

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

		self.log.info("Simulation of {0:.2f}s (ST)".format(st) + \
			" finished in {0:.2f}s (RT)".format(rt) + \
			" with acceleration of {0:.3f} x".format(st/rt))

	def load(self, filename):
		"""
		Load itself from a pickle file
		"""
		f = open(filename,'rb')
		tmp_dict = pkl.load(f)
		f.close()

		self.__dict__.update(tmp_dict.__dict__)

	def save(self, filename):
		"""
		Save class and variables with pickle
		"""

		f = open(filename,'wb')
		pkl.dump(self, f, 2)
		f.close()

	def plot(self, filename="history.png"):

		plt.plot(np.array(self.hist_cmd))
		plt.savefig(filename, format='png', dpi=300)
		plt.close()


class Sine(Controller):

	def __init__(self, config):

		super(Sine, self).__init__(config)

		self.n_motors = 4

		self.norm_f = 2 * math.pi * self.params[0]["f"]
		self.norm_a = self.params[0]["a"]
		self.norm_phi = self.params[0]["phi"]

	def set_norm_params(self, liste):

		j = 0
		params = []
		for i in range(self.n_motors):
			f = liste[j] * self.norm_f
			a = liste[j+1] * self.norm_a
			phi = liste[j+2] * self.norm_phi
			params.append({"f": f, "a": a, "phi": phi})
			j += 3

		self.params = params

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
				self.params[i]["f"] * self.t + self.params[i]["phi"]))
		
		self.hist_cmd.append(cmd)
		return cmd


class CPG(Controller):


	def __init__(self, config):

		super(CPG, self).__init__(config)

		self.n_motors = 4

		self.r = [float(self.params[i]["mu"]) for i in range(self.n_motors)]
		self.phi = [np.pi * float(self.params[i]["duty_factor"]) for i in range(self.n_motors)]
		self.o = [float(self.params[i]["o"]) for i in range(self.n_motors)]

		self.kappa_r = [1] * 4
		self.kappa_phi = [1] * 4
		self.kappa_o = [1] * 4
		self.f_r = [0] * 4
		self.f_phi = [0] * 4
		self.f_o = [0] * 4

		self.dt = float(self.config["Controller"]["integ_time"])
		self.gamma = 0.01
		self.prev_t= -1

		self.coupling = [self.params[i]["coupling"] for i in range(self.n_motors)]
		a, b = float(self.params[1]["phase_offset"]), float(self.params[2]["phase_offset"])
		c = float(self.params[3]["phase_offset"])
		d = a-b
		e = a-c
		f = b-c
		self.psi = [[0, a, b, c],
					[-1*a, 0, d, e],
					[-1*b, -1*d, 0, f],
					[-1*c, -1*e, -1*f, 0]]

	def step_cpg(self):

		cmd = []

		for i in range(self.n_motors):

			# Fetch current motor values
			dt = self.dt
			gamma = self.gamma

			mu = float(self.params[i]["mu"])
			omega = float(self.params[i]["omega"])
			d = float(self.params[i]["duty_factor"])
			
			r = self.r[i]
			phi = self.phi[i]
			o = self.o[i]

			kappa_r = self.kappa_r[i]
			kappa_phi = self.kappa_phi[i]
			kappa_o = self.kappa_o[i]
			f_r = self.f_r[i]
			f_phi = self.f_phi[i]
			f_o = self.f_o[i]
			cpl = self.coupling[i]
			 
			# Compute step evolution of r, phi and o
			d_r = gamma * (mu + kappa_r * f_r - r * r) * r
			d_phi = omega + kappa_phi * f_phi
			d_o = kappa_o * f_o

			# Add phase coupling
			for j in range(self.n_motors):
				d_phi += cpl[j] * np.sin(self.phi[j] - phi - self.psi[i][j])

			# Update r, phi and o
			self.r[i] += dt * d_r
			self.phi[i] += dt * d_phi
			self.o[i] += dt * d_o

			# Threshold phi to 2pi max
			phi_thr = 0
			phi_2pi = self.phi[i] % (2 * math.pi)
			if phi_2pi < ((2 * math.pi) * d):
				phi_thr = phi_2pi / (2 * d)
			else:
				phi_thr = (phi_2pi + (2 * math.pi) * (1 - 2 * d)) / (2 * (1 - d))

			# Save action
			action = self.r[i] * np.cos(phi_thr) + self.o[i]
			cmd.append(action)

		self.hist_cmd.append(cmd)
		return cmd

	def step(self, t):

		self.t = t
		n_steps = (int(self.t/self.dt) - self.prev_t)
		cmd = []

		for _ in range(n_steps):
			cmd = self.step_cpg()

		self.prev_t = int(self.t/self.dt)

		return cmd


if __name__ == '__main__':

	# Test Sine evolution
	config = {"Controller":
				{"params": "[{'f': 6, 'a': 0.4, 'phi': 0}, \
						{'f': 6, 'a': 0.4, 'phi': 0},\
						{'f': 6, 'a': 0.4, 'phi': 3.14},\
						{'f': 6, 'a': 0.4, 'phi': 3.14}]",
				"timestep": 0.001,
				"simtime": 5}}
	t = 0
	dt = config["Controller"]["timestep"]
	st = config["Controller"]["simtime"]
	n = int(st / dt)
	sc = Sine(config)

	t_init = time.time()
	for i in range(n): 
		sc.step(t)
		t += dt
	t_tot = time.time() - t_init
	print(str(n) + " iterations computed in " + str(t_tot) + " s")

	sc.plot("sine.png")

	# Test CPG evolution
	config = {"Controller":
				{"params": "[{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.6, 'phase_offset': 0, 'coupling': [5,5,5,0]}, \
		 					{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.6, 'phase_offset': 6.28, 'coupling': [5,5,5,0]},\
							{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.9, 'phase_offset': 3.14, 'coupling': [5,5,5,0]},\
		 					{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.9, 'phase_offset': 3.14, 'coupling': [5,5,5,0]}]",
				"integ_time": 0.001,
				"timestep": 0.01,
				"simtime": 10}}

	t = 0
	dt = config["Controller"]["timestep"]
	st = config["Controller"]["simtime"]
	n = int(st / dt)
	sc = CPG(config)

	t_init = time.time()
	for i in range(n): 
		sc.step(t)
		t += dt
	t_tot = time.time() - t_init
	print(str(n) + " iterations computed in " + str(t_tot) + " s")

	sc.plot("cpg.png")

