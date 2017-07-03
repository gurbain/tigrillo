
"""
This file is the top executable file. It acts like the orchest conductor by
running an experimental pipe and saving the results.
"""

import logging
import ast

from control import *
from optim import *
from physics import *
from utils import *

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "0.1" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 20th, 2017"


class Experiment():
	"""
	A class that run the whole pipe of an experiment given from a certain 
	config file.
	"""
	
	def __init__(self, config):

		# Retrieve the pipe
		self.config = config
		self.pipe = ast.literal_eval(config.get("Experiment", "pipe"))

		# Configure the log file
		self.log = logging.getLogger('Experiment')

		# Log all the scope of experiment
		self.log.info(getDateString())
		self.log.info(getGitString())
		self.log.info(getMachineString())
		self.log.info(getGPUString())
		self.log.info(getOSString())
		self.log.info(getPythonString())
		self.log.info(getConfigString(self.config))

		# Experiment parameters
		self.mode = None
		self.simtime = 0

		# Folder for saving results
		if "result_folder" in config["Experiment"]:
			self.result_folder = config.get("Experiment", "result_folder")
		else:
			self.result_folder = "results"
		mkdir(self.result_folder)

	def start(self):
		"""
		Run the whole pipe
		"""

		for params in self.pipe:
			self.phys = eval(params["phys"])(self.config)
			self.cont = eval(params["ctrl"])(self.config)
			self.start_seq(params)

	def start_seq(self, params):
		"""
		Run a full sequence
		"""

		# Set-up
		self.mode = params["mode"]
		self.simtime = params["time"]
		

		if self.mode == "run":
			self.phys.start_sim()
			self.cont.run(self.simtime, self.phys)
			self.phys.stop_sim()

		if self.mode == "optim":
			opt = Optim(self.simtime, self.phys, self.cont, self.config)
			params, score, duration = opt.run()
			opt.save()
			self.cont.set_norm_params(params)

		self.save()

	def save(self):
		"""
		Save all usefull information to analyse the experiment
		and reproduce it later
		"""
		folder = self.result_folder + '/' + time.strftime("%Y%m%d-%H%M%S" + "/")
		mkdir(folder)

		# self.phys.save()
		# save optim params..
		self.cont.save(folder + "best_cont.pkl")
		cp(self.config.get("Config", "filename"), folder + "config.conf")

		return


