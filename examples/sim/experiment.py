
"""
This file is the top executable file. It acts like the orchest conductor by
running an experimental pipe and saving the results.
"""

import ast

from tigrillo.core.physics import *
from tigrillo.core.control import *
from tigrillo.core.utils import *
from tigrillo.core.optim import *

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 20th, 2017"


class Experiment:
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
		self.log.info(get_pid_string())
		self.log.info(get_date_string())
		self.log.info(get_git_string())
		self.log.info(get_machine_string())
		# self.log.info(get_gpu_string())
		self.log.info(get_os_string())
		self.log.info(get_python_string())
		self.log.info(get_config_string(self.config))

		# Experiment parameters
		self.mode = None
		self.sim_time = 0
		self.phys = None
		self.cont = None
		self.optim = None

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
		self.sim_time = params["time"]

		if self.mode == "run":
			self.phys.start_sim()
			self.cont.run(self.sim_time, self.phys)
			self.phys.stop_sim()

		if self.mode == "optim":
			self.optim = Optim(self.sim_time, self.phys, self.cont, self.config)
			params, score, duration = self.optim.run()
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
		if self.mode == "optim":
			self.cont.save(folder + "best_cont.pkl")
			self.optim.save(folder + "cma.csv")
			self.optim.plot(folder + "cma.png")

		cp(self.config.get("Config", "filename"), folder + "config.conf")

		return
