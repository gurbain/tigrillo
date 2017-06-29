"""
Contains the files for optimization algorithms
"""

import logging
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pickle
import random
import time
import sys

import cma


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "0.1" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 14th, 2017"


class Score():

	def __init__(self, physics, controller, config):

		# Retrieve parameters
		self.phys = physics
		self.cont = controller
		self.config = config
		params = config["Optim"]

		if "score" in params:
			self.score_type = params["score"]
		else:
			self.score_type = "distance"

	def start(self):

		# Take a measurement in the initial state
		self.init_pose_state = self.phys.sim_model_state
		self.init_time = self.phys.sim_duration

	def stop(self):

		# Take a measurement in the final state
		self.final_pose_state = self.phys.sim_model_state
		print(self.final_pose_state)
		self.final_time = self.phys.sim_duration

	def get_score(self):

		if self.score_type == "distance":
			return self.get_dist_score()

	def get_dist_score(self):

		return math.sqrt((self.init_pose_state["pos"]["x"] - \
			self.init_pose_state["pos"]["x"]) ** 2 + \
			(self.init_pose_state["pos"]["y"] - \
			self.init_pose_state["pos"]["y"]) ** 2)

class Optim():

	def __init__(self, simtime, physics, controller, config):

		# Retrieve parameters
		self.phys = physics
		self.cont = controller
		self.config = config
		params = config["Optim"]

		# Configure the log file
		self.log = logging.getLogger('Optim')

		if "max_iter" in params:
			self.max_iter = float(params["max_iter"])
		else:
			self.max_iter = 500
		if "init_mean" in params:
			self.init_mean = float(params["init_mean"])
		else:
			self.init_mean = 0.5
		if "init_var" in params:
			self.init_var = float(params["init_var"])
		else:
			self.init_var = 0.2


		self.it = 0
		self.simtime = simtime
		self.dim = self.cont.get_params_len()

		self.score = Score(self.phys, self.cont, self.config)
		self.hist_score = []

	def run(self):

		# Init algorithm
		es = cma.CMAEvolutionStrategy(self.dim * [self.init_mean], self.init_var, 
		{'boundary_handling': 'BoundTransform ','bounds': [0,1], 
		'maxfevals' : self.max_iter,'verbose' :-9})
		self.pop_size = es.popsize
		t_init = time.time()

		# Run optimization
		print("== Start Optimization process with dim of " + str(self.dim) + \
			" and population size of " + str(self.pop_size) + " ==\n")
		while not es.stop():
			solutions = es.ask()
			es.tell(solutions, [self.evaluateParam(list) for list in solutions])
		t_stop = time.time()
		res = es.result();
		
		# Return optimum
		self.opt_time = t_stop - t_init
		self.opt_param = res[0]
		self.opt_score = -res[1]
		print("== Finish Optimization process with opt score = " + \
			"{0:.3f} and params = ".format(self.opt_score) + \
			str(self.opt_param) + " == ")
		print("== Optimization time for " + str(self.it) + \
				" epochs: {0:.1f}s.".format(self.opt_time) + \
				" {0:.3f}s in average per iteration ==".format(self.opt_time/self.it))
		return self.opt_param, self.opt_score, self.opt_time

	def evaluateParam(self, liste):

		# Init
		self.cont.set_norm_params(liste)
		self.score.start()
		t_init = time.time()

		# Run
		self.phys.start_sim()
		self.cont.run(self.simtime, self.phys)

		# Stop
		self.score.stop()
		st = self.score.final_time
		t_fin = time.time()
		self.phys.stop_sim()
		rt = t_fin - t_init

		# Get score
		score = self.score.get_score()
		self.hist_score.append(score)
		
		self.it += 1
		print("it " + str(self.it) + ": score = {0:.3f}".format(score) +
			" (rt = {0:.2f}s; ".format(rt) + \
			"st = {0:.2f}s; ".format(st) + \
			"acc: {0:.2f}X)".format(st/rt))

		return -score

	def save(self):

		#save self.hist_score
		return

	def plot(self):

		plt.plot(self.hist_score)
		plt.savefig("score.png", format='png', dpi=300)
		plt.close()


def optimize(n_it):

	# Optimize
	n_it = int(n_it)
	opt = Optim(n_it)
	params, score, duration = opt.run()
	namefile = time.strftime("%Y%m%d-%H%M%S", time.localtime())
	pickle.dump(params, open("results/" + namefile + ".pkl", "wb"))
	opt.plot()
	print("== Optimization time for " + str(n_it) + \
		" epochs: {0:.1f}s. ".format(duration) + \
		" {0:.3f}s in average per iteration ==".format(duration/n_it))