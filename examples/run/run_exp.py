
"""
This execute an experiment in realtime on the robot 
"""

from tigrillo.core.control import *
from tigrillo.core.utils import *

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 15th, 2017"


class Experiment:
	"""
	A class that run the whole pipe of an experiment given from a certain 
	config file.
	"""
	
	def __init__(self, configuration):

		# Retrieve the pipe
		self.config = configuration
		self.pipe = ast.literal_eval(config.get("Experiment", "pipe"))

		# Configure the log file
		self.log = logging.getLogger('Experiment')

		# Log all the scope of experiment
		self.log.info(get_date_string())
		self.log.info(get_git_string())
		self.log.info(get_machine_string())
		self.log.info(get_gpu_string())
		self.log.info(get_os_string())
		self.log.info(get_python_string())
		self.log.info(get_config_string(self.config))

		# Experiment parameters
		self.mode = None
		self.sim_time = 0

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

		# To implement

	def save(self):
		"""
		Save all usefull information to analyse the experiment
		and reproduce it later
		"""

		# To implement
