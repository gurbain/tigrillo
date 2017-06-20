
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

	def run(self):
		"""
		Run the whole pipe
		"""

		for params in self.pipe:
			self.run_seq(params)

	def run_seq(self, params):
		"""
		Run a full sequence
		"""

		physics = ast.literal_eval(params["phys"])(self.config)
		controller = ast.literal_eval(params["phys"])(self.config)

		if params["mode"] == "run" and params["conn"] == "ol":


