#!/usr/bin/python3

"""
This script plot the trajectory performed by the real robot on the ground
"""


import tigrillo
from tigrillo.core import utils

import logging
import matplotlib
import os
import sys

matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.style.use('fivethirtyeight')
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.rc('axes', facecolor='white')
plt.rc('savefig', facecolor='white')
plt.rc('lines', linewidth=0.8)


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 15th, 2017"

DEFAULT_SENSOR_FILE = os.path.join(tigrillo.__path__[0], '../results/run/sensors_20170921-133955.csv')
DEFAULT_IMU_ANALYSIS_FOLDER = os.path.join(tigrillo.__path__[0], '../results/analysis/imu')


class IMUAnalysis(object):

    def __init__(self, configuration):

        self.imu_results = dict()
        self.t_step = 0

        # Retrieve the config
        self.config = configuration
        self.log = logging.getLogger('IMUAnalysis')
        params = dict()

        if "IMUAnalysis" in self.config:
            params = configuration["IMUAnalysis"]

        if "folder" in params:
            self.folder = params["folder"]
        else:
            self.folder = DEFAULT_IMU_ANALYSIS_FOLDER
        utils.mkdir(self.folder)

        if "analysis_file" in params:
            self.load(params["analysis_file"])

    def __str__(self):

        st = ""
        for key, value in sorted(self.imu_results.iteritems()):
            if len(key) < 16:
                st += "\n" + str(key) + ":\t\t"
            else:
                st += "\n" + str(key) + ":\t"
            for i in range(min([10, len(value)])):
                n = value[i]
                if (abs(n) < 0.001 and abs(n) > 0.00000001) or abs(n) > 1E5:
                    st += "%.2e  " % n
                else:
                    st += "%.4f  " % n
            st += " ..."
        return st

    def load(self, filename):

        self.imu_results = utils.load_csv(filename)
        self.log.info("Loading file" + str(filename))
        self.t_step = self.imu_results["Run Time: "][1] - self.imu_results["Run Time: "][0]

    def plot_accelerations(self, filename="imu_acc", t_end=5, title=None, show=False, save=True):
        """ Plot the IMU accelerations """

        self.log.info("Printing IMU accelerations")
        i_last = int(t_end / self.t_step)

        fig, ax = plt.subplots()

        ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Linear Acceleration X"][0:i_last],
                label="X acceleration")
        ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Linear Acceleration Y"][0:i_last],
                label="Y acceleration")
        # ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Linear Acceleration Z"][0:i_last],
        # label="Z acceleration")

        if title:
            plt.title(title)
        plt.xlabel('Running time [s]')
        plt.ylabel('Robot Accelerations [$m / s^2$]')
        plt.tight_layout()
        if show:
            plt.show()
        if save:
            plt.savefig(self.folder + "/" + filename + ".png", format='png', dpi=300)
        plt.close()

    def plot_positions(self, filename="imu_pos", t_end=5, title=None, show=False, save=True):
        """ Plot the IMU positions """

        self.log.info("Printing IMU positions")
        i_last = int(t_end / self.t_step)

        fig, ax = plt.subplots()

        ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Position X"][0:i_last],
                label="X position")
        ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Position Y"][0:i_last],
                label="Y position")
        ax.plot(self.imu_results["Run Time: "][0:i_last], self.imu_results["Position Z"][0:i_last],
                label="Z position")

        if title:
            plt.title(title)
        plt.xlabel('Running time [s]')
        plt.ylabel('Robot Position [m]')
        plt.tight_layout()
        if show:
            plt.show()
        if save:
            plt.savefig(self.folder + "/" + filename + ".png", format='png', dpi=300)
        plt.close()


if __name__ == '__main__':

    # Configuration
    args = sys.argv
    conf = utils.set_default_logger(shell_verbose=False, file_verbose=False)
    log = logging.getLogger('Main')

    if len(args) > 1:
        imu_file = args[1]
    else:
        log.info("No argument given. Using file" + str(DEFAULT_SENSOR_FILE) + " by default.")
        imu_file = DEFAULT_SENSOR_FILE

    # Create analysis object
    imua = IMUAnalysis(conf)
    imua.load(imu_file)

    # Print Sensor values
    log.warning(imua)

    # Plot data evolutions
    imua.plot_accelerations()
    imua.plot_positions()
