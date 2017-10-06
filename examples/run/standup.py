"""
Script to perform open loop run using hand-made splined with the Tigrillo Robot and save all sensors data
"""


from tigrillo.rpi import robot
from tigrillo.core.timing import Timer

import time

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 12th, 2017" 


RESULTS_FOLDER = "../results/run/"


if __name__ == '__main__':

    # Create and configure robot (by defaut: save actuators and sensors values)
    rob = robot.Tigrillo(data_folder=RESULTS_FOLDER)
    rob.start()
    rob.set_sensor_period(2)

    # Create stand-up values (offset of 115 degrees)
    pose = {"FL": 80, "FR": 80, "BL": 60, "BR": 60}

    # Create a Timer to ensure that simulation time goes at real-time
    t = Timer(real_time=True, runtime=40, dt=0.1)
    t.start()

    # Perform experiment
    while not t.is_finished():

        rob.update_actuators(pose, t.st)
        t.update()
