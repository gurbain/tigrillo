"""
Contains the files for optimization algorithms
"""

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


class Optim():

	def __init__(self, max_iter=500, init_mean=0.5, init_std=0.2):
		self.init_mean = init_mean
		self.max_iter = max_iter
		self.init_std = init_std

		self.cont = SineController()
		self.dim = self.cont.getLen()
		self.it = 0

		self.hist_score = []

	def run(self):

		# Init algorithm
		es = cma.CMAEvolutionStrategy(self.dim * [self.init_mean], self.init_std, 
		{'boundary_handling': 'BoundTransform ','bounds': [0,1], 
		'maxfevals' : self.max_iter,'verbose' :-9})
		self.pop_size = es.popsize
		t_init = time.time()

		# Run optimization
		print("\n== Start Optimization process with dim of " + str(self.dim) + \
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
		print("\n== Finish Optimization process with opt score = " + \
			"{0:.3f} and params = ".format(self.opt_score) + \
			str(self.opt_param) + " == ")
		return self.opt_param, self.opt_score, self.opt_time

	def evaluateParam(self, liste):

		# Perform simulation with parametrized controller
		self.cont.setNormalized(liste)
		sim = Simulation(self.cont, rt=0, mode="direct", robust=1, model="models/tigrillo.sdf")
		sim.reset()
		sim.run()

		# Get perf value and close simulation
		score = sim.score()
		self.hist_score.append(score)
		rt = sim.getRealtime()
		st = sim.getSimtime()
		sim.stop()

		# Print
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