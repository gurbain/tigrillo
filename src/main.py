
"""
This file is the main example to run. It parses the command line and 
load the configuration file for the simulation.
"""

import argparse
import ast
import configparser
import logging
import time

from experiment import *
from utils import *

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "0.1" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 20th, 2017"


if __name__ == "__main__":
	""" Parse the arguments, config file and run all """

	# Parse arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("-c", "--config", type=str, help="The " + \
		" configuration file that contains all the parameters " + \
		" for the experiment.", required=True)
	parser.add_argument("-v", "--verbose", action="store_true", 
		help="Force shell verbosity to INFO despite of config file entry.")
	parser.add_argument("-l", "--log", action="store_true", 
		help="Force file logs verbosity to DEBUG despite of config file entry.")
	parser.add_argument("-x", "--gui", action="store_true", 
		help="Force the GUI despite of config file entry.")
	args = parser.parse_args()
	if not args.config:
		parser.error("No config file given. Please, add a config file " +  \
			"after the -c option")

	# Parse config file
	config = configparser.ConfigParser()
	config.read(args.config)

	# Manually change forced command-line arguments
	if args.gui:
		config.set("Simulation", "rendering", "True")
	if args.verbose:
		config.set("Logger", "level_shell", "INFO")
	if args.log:
		config.set("Logger", "level_file", "DEBUG")

	# Retrieve logging parameters
	log_file_level = eval("logging." + \
		config.get("Logger", "level_file"))
	log_shell_level = eval("logging." + \
		config.get("Logger", "level_shell"))
	log_file_folder = config.get("Logger", "folder")
	mkdir(log_file_folder)
	log_file_name = log_file_folder + "/" + time.strftime("%Y%m%d_%H%M%S", \
		time.localtime()) + ".log"

	# Set up logger
	logging.basicConfig(level=log_file_level,
		format='[%(asctime)s - %(levelname)-8s: %(name)s]  %(message)s',
		datefmt='%y-%m-%d %H:%M:%S',
		filename=log_file_name,
		filemode='w')
	log_shell = logging.StreamHandler()
	log_shell.setLevel(log_shell_level)
	log_shell_format = logging.Formatter("[%(name)-12s: %(levelname)-8s] " + \
		" %(message)s")
	log_shell.setFormatter(log_shell_format)
	logging.getLogger('').addHandler(log_shell)

	# Run the experiment
	e = Experiment(config)
	e.run()