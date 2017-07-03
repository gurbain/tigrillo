
"""
This file contains all third-party methods
"""

import datetime
import git
import logging
from mpi4py import MPI as mpi
import os
import platform
import pyopencl
from shutil import copyfile
import threading


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "0.1" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "February 22nd, 2017"


def mkdir(path):
	""" Create a directory if it does not exist yet """

	if not os.path.exists(path):
		os.makedirs(path)

def cp(src, dst):

	copyfile(src, dst)

def getDateString():
	""" Return a string with datetime """

	return "Date: " + str(datetime.datetime.now())

def getOSString():
	""" Return a string with datetime """

	string = "OS: " + str(platform.system()) + " version "
	string += str(platform.release())
	return string

def getGitString():
	""" Return a string with information on the git version """

	repo = git.Repo(search_parent_directories=True)
	sha = repo.head.object.hexsha

	return "Git branch hash: " + sha

def getPythonString():
	""" Return a string with python information """
	
	return "Python version: " + str(platform.python_version ())

def getMachineString():
	""" Return a string with machine and MPI information """

	comm = mpi.COMM_WORLD
	rank = comm.Get_rank()
	size = comm.Get_size()
	machine = platform.node()
	string = "Machine: " + machine + " (" + str(rank+1) + "/" +  \
		str(size) + ")"
	return string

def getGPUString():
	""" Return a string with all MPI information """

	platforms = pyopencl.get_platforms()
	for p in platforms:
		string = "OpenCL: " + p.name + " (" + p.version + ") with devices: "
		devices = p.get_devices()
		for d in devices:
			string += d.name + "  "
	return string

def getConfigString(config):
	""" Return a string with the config file """

	string = "Config:\n\n"
	for sec in config.sections():
		string += "\n[" + sec + "]"
		for (key, val) in config.items(sec):
			string += "\n" + key + "=" + val
	return string + "\n\n"

class LogPipe(threading.Thread):

	def __init__(self, name):
		"""Setup the object with a logger and a loglevel
		and start the thread
		"""
		threading.Thread.__init__(self)
		self.daemon = False

		self.log = logging.getLogger(name)

		self.fdRead, self.fdWrite = os.pipe()
		self.pipeReader = os.fdopen(self.fdRead)
		self.start()

	def fileno(self):
		"""Return the write file descriptor of the pipe
		"""
		return self.fdWrite

	def run(self):
		"""Run the thread, logging everything.
		"""
		for line in iter(self.pipeReader.readline, ''):
			self.log.info(line.strip('\n'))

		self.pipeReader.close()

	def close(self):
		"""Close the write end of the pipe.
		"""
		os.close(self.fdWrite)