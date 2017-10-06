
"""
This file contains all third-party methods
"""

import configparser
import csv
import datetime
import git
import logging
from mpi4py import MPI as mpi
import os
import platform
from shutil import copyfile
import tigrillo
import time
import threading


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "February 22nd, 2017"


# Files and IOs Utils

default_logger = False
default_logger_config_file = os.path.join(tigrillo.__path__[0], '../data/configs/default_log.conf')
default_logger_folder = os.path.join(tigrillo.__path__[0], '../results/log/')


def mkdir(path):
    """ Create a directory if it does not exist yet """

    if not os.path.exists(path):
        os.makedirs(path)


def cp(src, dst):
    """ Copy a file in another """

    copyfile(src, dst)


def load_csv(path):
    """ Load data from a csv file """

    if not path.endswith('.csv'):
        raise ValueError('CSV files should end with .csv, but got %s instead' % path)

    with open(path, mode='r') as infile:
        reader = csv.DictReader(infile)
        result = {}
        for row in reader:
            for column, value in row.iteritems():
                result.setdefault(column, []).append(float(value))

    return result


# Information printing utils

def timestamp():
    """ Return a string stamp with current date and time """

    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def get_pid_string():
    """ Return a string with process PID """

    return "PID: " + str(os.getpid())


def get_date_string():
    """ Return a string with datetime """

    return "Date: " + str(datetime.datetime.now())


def get_os_string():
    """ Return a string with datetime """

    string = "OS: " + str(platform.system()) + " version "
    string += str(platform.release())
    return string


def get_git_hash():
    """ Return the current git hash """

    repo = git.Repo(search_parent_directories=True)
    return repo.head.object.hexsha


def get_git_string():
    """ Return a string with information on the git version """

    sha = get_git_hash()
    return "Git branch hash: " + sha


def get_python_string():
    """ Return a string with python information """

    return "Python version: " + str(platform.python_version())


def get_machine_string():
    """ Return a string with machine and MPI information """

    comm = mpi.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()
    machine = platform.node()
    string = "Machine: " + machine + " (" + str(rank+1) + "/" + str(size) + ")"

    return string


def get_gpu_string():
    """ Return a string with all MPI information """

    platforms = pyopencl.get_platforms()
    string = ""
    for p in platforms:
        string = "OpenCL: " + p.name + " (" + p.version + ") with devices: "
        devices = p.get_devices()
        for d in devices:
            string += d.name + "  "
    return string


def get_config_string(config):
    """ Return a string with the config file """

    string = "Config:\n\n"
    for sec in config.sections():
        string += "\n[" + sec + "]"
        for (key, val) in config.items(sec):
            string += "\n" + key + "=" + val
    return string + "\n\n"


def set_default_logger(shell_verbose=False, file_verbose=False):
    """ The Tigrillo software works with a config file that sets, among other, a file and shell logger.
    Instead of using the full config file, the user can just call this function to set a default logger only. """

    global default_logger

    if not default_logger:
        default_logger = True

        # Create config
        config = configparser.ConfigParser()
        config.add_section('Config')
        config.set('Config', 'filename', os.path.abspath("no_file"))

        config.add_section('Logger')
        config.set("Logger", "folder", default_logger_folder)
        if shell_verbose:
            config.set("Logger", "level_shell", "INFO")
        else:
            config.set("Logger", "level_shell", "WARNING")
        if file_verbose:
            config.set("Logger", "level_file", "DEBUG")
        else:
            config.set("Logger", "level_file", "INFO")

        # Retrieve logging parameters
        log_file_level = eval("logging." + config.get("Logger", "level_file"))
        log_shell_level = eval("logging." + config.get("Logger", "level_shell"))
        log_file_folder = config.get("Logger", "folder")
        mkdir(log_file_folder)
        log_file_name = log_file_folder + "/" + time.strftime("%Y%m%d_%H%M%S", time.localtime()) + ".log"

        # Set up logger
        logging.basicConfig(level=log_file_level,
                            format='[%(levelname)-12s' + get_git_hash()[-8:] +
                                   ' - %(filename)s - l%(lineno)s - %(name)s - %(asctime)s]  %(message)s',
                            datefmt='%y-%m-%d %H:%M:%S',
                            filename=log_file_name,
                            filemode='w')
        log_shell = logging.StreamHandler()
        log_shell.setLevel(log_shell_level)
        log_shell_format = logging.Formatter("[%(levelname)-8s %(filename)s - %(name)s] %(message)s")
        log_shell.setFormatter(log_shell_format)
        logging.getLogger('').addHandler(log_shell)

        return config


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
