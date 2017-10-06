"""
Contains the files for optimization algorithms
"""

from tigrillo.core import cma

from copy import deepcopy
import logging
import math
import matplotlib.pyplot as plt
import time
import matplotlib
matplotlib.use("Agg")


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 14th, 2017"


class Score(object):

    def __init__(self, physics, controller, config):

        # Retrieve parameters
        self.phys = physics
        self.cont = controller
        self.config = config
        params = config["Optim"]

        self.init_pose_state = None
        self.init_time = None
        self.final_pose_state = None
        self.final_time = None

        if "score" in params:
            self.score_type = params["score"]
        else:
            self.score_type = "distance"

    def start(self):

        # Take a measurement in the initial state
        self.init_pose_state = deepcopy(self.phys.sim_model_state)
        self.init_time = deepcopy(self.phys.sim_duration)

    def stop(self):

        # Take a measurement in the final state
        self.final_pose_state = self.phys.sim_model_state
        self.final_time = self.phys.sim_duration

    def get_score(self):

        if self.score_type == "distance":
            return self.get_dist_score()

    def get_dist_score(self):

        dist = math.sqrt((self.final_pose_state["pos"]["x"] -
                          self.init_pose_state["pos"]["x"]) ** 2 +
                         (self.final_pose_state["pos"]["y"] -
                          self.init_pose_state["pos"]["y"]) ** 2)

        return dist


class Optim(object):

    def __init__(self, sim_time, physics, controller, config):

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
        self.sim_time = sim_time
        self.dim = self.cont.get_params_len()
        self.pop_size = 0
        self.opt_time = 0
        self.opt_param = None
        self.opt_score = 0

        self.score = Score(self.phys, self.cont, self.config)
        self.hist_score = []

    def run(self):

        # Add constraints here

        # Init algorithm
        es = cma.CMAEvolutionStrategy(self.dim * [self.init_mean], self.init_var,
                                      {'boundary_handling': 'BoundTransform ', 'bounds': [0, 1],
                                       'maxfevals': self.max_iter, 'verbose': -9})
        self.pop_size = es.popsize
        t_init = time.time()

        # Run optimization
        self.log.info("== Start Optimization process with dim of " + str(self.dim) +
              " and population size of " + str(self.pop_size) + " ==\n")
        while not es.stop():
            solutions = es.ask()
            es.tell(solutions, [self.evaluate_param(l) for l in solutions])
        t_stop = time.time()
        res = es.result()

        # Return optimum
        self.opt_time = t_stop - t_init
        self.opt_param = res[0]
        self.opt_score = -res[1]
        print("== Finish Optimization process with opt score = " +
              "{0:.3f} and params = ".format(self.opt_score) +
              str(self.opt_param) + " == ")
        print("== Optimization time for " + str(self.it) +
              " epochs: {0:.1f}s.".format(self.opt_time) +
              " {0:.3f}s in average per iteration ==".format(self.opt_time/self.it))
        return self.opt_param, self.opt_score, self.opt_time

    def evaluate_param(self, l):

        # Init
        self.cont.set_norm_params(l)
        self.score.start()
        t_init = time.time()

        # Run
        self.phys.start_sim()
        self.cont.run(self.sim_time, self.phys)
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
              " (rt = {0:.2f}s; ".format(rt) +
              "st = {0:.2f}s; ".format(st) +
              "acc: {0:.2f}X)".format(st/rt))

        return -score

    def save(self, filename):

        # save here

        return

    def plot(self, filename="score.png"):

        plt.plot(self.hist_score)
        plt.savefig(filename, format='png', dpi=300)
        plt.close()
