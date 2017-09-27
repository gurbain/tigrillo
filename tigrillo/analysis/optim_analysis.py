#!/usr/bin/python3

"""
This script opens a windows to analyze various properties of the optimization results
"""

import configparser

from tigrillo.core.control import *
from tigrillo.core.optim import *


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "July 3rd, 2017" 


class OptimAnalysis:

    def __init__(self):

        self.result_folder = None
        self.config = None
        self.phys = None
        self.sim_time = None
        self.cont = None
        self.score = None

    def load(self, folder):

        self.result_folder = folder

        # Retrieve config file
        self.config = configparser.ConfigParser()
        self.config.read(self.result_folder + "config.conf")

        self.config.set('Physics', 'rendering',  "True")

        # TODO: stop taking the first pipe result by default
        param = ast.literal_eval(self.config.get("Experiment", "pipe"))[0]
        self.phys = eval(param["phys"])(self.config)

        self.sim_time = param["time"]
        self.cont = eval(param["ctrl"])(self.config)

    def simulate(self):

        # Init
        self.cont.load(self.result_folder + "/best_cont.pkl")
        self.score = Score(self.phys, self.cont, self.config)
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

        print("Simulation finished with score = {0:.3f}".format(score) +
              " (rt = {0:.2f}s; ".format(rt) +
              "st = {0:.2f}s; ".format(st) +
              "acc: {0:.2f}X)".format(st/rt))


if __name__ == '__main__':

    an = OptimAnalysis()
    an.load("/home/gabs48/tigrillo/data/results/20170703-165722/")

    an.simulate()
