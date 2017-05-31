
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pickle
import pybullet as p
import random
import time
import sys

import cma

class SineController():

	def __init__(self):
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


class Simulation():

	def __init__(self, controller, rt=1, mode="gui", robust=0, model="models/tigrillo.sdf"):

		self.t_step = 0.001
		self.plane = "models/plane.urdf"
		self.t = 0
		self.t_max = 7
		self.dim = int(1./self.t_step) * self.t_max + 1
		self.cont = controller
		self.rt = rt
		self.mode = mode
		self.robust = robust
		self.model = model
		self.imp_av_inter_time = 1.5
		self.imp_time = 0.1
		self.noise_max = 3 #N (the robot weight 1kg so 10N are enough to lift it)

	def reset(self):

		# Init simulator
		if self.mode == "gui":
			p.connect(p.GUI)
		else:
			p.connect(p.DIRECT)
		p.loadURDF(self.plane, 0, 0, -1.5)
		p.setGravity(0, 0, -9.81)
		p.setTimeStep(self.t_step)
		self.robot = p.loadSDF(self.model)[0]

		# Create noise vector
		if self.robust == 1:
			self.createNoise()

		# Reset pose
		self.n_joint = p.getNumJoints(self.robot)
		j = 0
		for i in range(self.n_joint):
			name = p.getJointInfo(self.robot, i)[1].decode('UTF-8')
			if name.find("UJ") != -1:
				p.resetJointState(self.robot, i, self.cont.step(self.t, j, 0, 0))
				j += 1
			else:
				# Here we add a spring
				p.resetJointState(self.robot, i, 0)
			

		# Get initial states
		self.pos_start, self.ori_start = p.getBasePositionAndOrientation(self.robot)

	def createNoise(self):

		self.noise = np.zeros((self.dim, 3))
		n_imp = int(self.t_max / self.imp_av_inter_time)
		t_start = np.random.randint(0, self.dim, int(n_imp))
		t_imp = []
		for i in range(int(self.imp_time / self.t_step)):
			t_imp.extend(np.add(t_start, i).tolist())
		first = True
		for i in range(self.dim):
			if i in t_imp:
				if first:
					val = np.random.uniform(-self.noise_max, self.noise_max, 3)
					first = False
				self.noise[i] += val
			else:
				first = True

	def run(self):

		self.t_start = time.time()
		i = 0
		while self.t < self.t_max:
			self.t = self.t + self.t_step
			self.step(i)
			i += 1
		self.t_stop = time.time()

	def step(self, step):
		# Set controller values

		time_start_it = time.time()
		j = 0
		for i in range(self.n_joint):
			name = p.getJointInfo(self.robot, i)[1].decode('UTF-8')
			if name.find("UJ") != -1:
				pos_feed, speed_feed, r_f, t = p.getJointState(self.robot, i)
				p.setJointMotorControl2(self.robot, i, p.POSITION_CONTROL, \
					targetPosition=self.cont.step(self.t, j, 0, 0))
				j += 1

		# Apply noise
		if self.robust == 1 and self.noise[step][0] != 0:
			p.applyExternalForce(self.robot, -1, self.noise[step].tolist(), \
				[0, 0, 0], p.LINK_FRAME)

		# Run physics step
		if self.rt:
			while time.time() < time_start_it + 0.85 * self.t_step:
				time.sleep(self.t_step/100)
		p.stepSimulation()

	def stop(self):

		p.disconnect()

	def score(self):
		#print(p.getContactPoints(self.robot))
		pos, ori = p.getBasePositionAndOrientation(self.robot)
		x_dist = (pos[1] - self.pos_start[1]) - np.abs(pos[0] - self.pos_start[0])/10

		return x_dist

	def getSimtime(self):
		
		return self.t

	def getRealtime(self):
		
		return self.t_stop - self.t_start


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

def run(params):

	# Test
	cont = SineController()
	cont.setNormalized(params)
	sim = Simulation(cont, mode="gui", rt=1)
	sim.reset()
	sim.run()
	score = sim.score()
	rt = sim.getRealtime()
	st = sim.getSimtime()
	print("Score = {0:.3f}".format(score) +
			" (rt = {0:.2f}s; ".format(rt) + \
			"st = {0:.2f}s; ".format(st) + \
			"acc: {0:.2f}X)".format(st/rt))
	sim.stop()
	cont.plot()


if __name__ == "__main__":

	params = [0.50159364, 0.64696674, 0.45324102, 0.7197049, 0.6678639, 0.81435154, 0.31556605, 0.5297903, 0.192041]
	
	if len(sys.argv) == 1:
		run(params)
	elif len(sys.argv) == 2:
		if sys.argv[1] == "opt":
			optimize(100)
		elif sys.argv[1] == "run":
			run(params)
		else:
			print("Error: the argument you called does not exist. Available: opt or run")
	elif len(sys.argv) == 3:
		if sys.argv[1] == "opt":
			optimize(sys.argv[2])
		elif sys.argv[1] == "run":
			params = pickle.load(open(sys.argv[2], "rb"))
			run(params)
		else:
			print("Error: the argument you called does not exist. Available: opt or run")

	else:
		print("Error: too many arguments")