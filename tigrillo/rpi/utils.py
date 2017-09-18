 

"""
This file contains all other methods
"""

from collections import deque
import os
import re
import serial
import threading
import time


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 11th, 2017"


MAX_BUFFER_SIZE = 100
DATA_FOLDER = "results"


def timestamp():
	""" Return a string stamp with current date and time """

	return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def make_dir(dir):
	""" Create the father directory of a filename if it does not exists """

	if not os.path.exists(dir):
		os.makedirs(dir)


class UARTDaemon(threading.Thread):
	"""
	This UART daemon monitors the use of a serial IO channel and store all receeeived
	data in a buffer for asynchronous usage
	"""

	def __init__(self, port, baud):

		super(UARTDaemon, self).__init__()

		self.port = port
		self.baud = baud
		self.conn = None

		self.data_buffer = deque([])
		self.ack_buffer = deque([])

		self.stop = True
		self.read_period = 0.0001
		self.daemon = True

	def start(self):
		""" Initialize and configure serial port """

		self.conn = serial.Serial(self.port, self.baud)
		if not self.conn.isOpen():
			self.conn.open()
		self.stop = False

		super(UARTDaemon, self).start()

	def stop(self):
		""" Raise a stop flag to stop the daemon """

		self.stop = True
		self.conn.close()

	def readData(self):
		""" Read the last data line in the uart buffer """

		if self.data_buffer:
			return self.data_buffer.popleft()

	def readAck(self):
		""" Read the last ack line in the uart buffer """

		if self.ack_buffer:
			return self.ack_buffer.popleft()

	def write(self, line):
		""" Write a line in the uart channel"""

		self.conn.write(line)

	def run(self):
		""" Read and populate the buffer"""

		while not self.stop:
			line = self.conn.readline()
			line_array = filter(None, re.split('; |: |\r|\*|\n', line))
			# TODO: only update when enough
			print "bbbb" + str(line_array)
			if len(line_array) > 0:
				if line_array[0] == "[DATA]":
					self.data_buffer.append(line_array[1:])
				if line_array[0] == "[ACK]":
					self.data_buffer.append(line_array[1:])
			time.sleep(self.read_period)

