"""
Script to perform open loop run using hand-made splined with the Tigrillo Robot and save all sensors data
"""


from copy import copy
import datetime
import pause

from tigrillo.rpi import robot
from tigrillo.core.control import *


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 12th, 2017" 


RESULTS_FOLDER = "data" # TO CHANGE


if __name__ == '__main__':

    # Create and configure robot (by defaut: save actuators and sensors values)
    rob = robot.Tigrillo(data_folder=RESULTS_FOLDER)
    rob.start()

    # Create and configure CPG controller
    config = {"Controller":
                  {"params": "[{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.6, "
                             "'phase_offset': 0, 'coupling': [5,5,5,0]},"
                             "{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.6, "
                             "'phase_offset': 6.28, 'coupling': [5,5,5,0]},"
                             "{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.9, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]},"
                             "{'mu': 0.4, 'o': 0, 'omega': 6.35, 'duty_factor': 0.9, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]}]",
                   "integ_time": 0.001,
                   "timestep": 0.02,
                   "runtime": 12}}
    ctl = CPG(config)

    # Set time variables
    t_real = datetime.datetime.now()
    t_run = 0
    t_real_init = copy(t_real)
    t_run_init = copy(t_run)
    dt = config["Controller"]["timestep"]
    t_run_end = config["Controller"]["runtime"]
    t_real_end = t_real + datetime.timedelta(seconds=t_run_end)
    iteration = 0

    # Perform experiment
    while (t_run < t_run_end) and (t_real < t_real_end):

        # Produce actuators control signal for open loop
        cmd = ctl.step(t_run)

        # Translate command representation
        update = {"FL": cmd[0], "FR": cmd[1], "BL": cmd[2], "BR": cmd[3]}

        # Update to the robot (to save call both actuators and sensors)
        measure = rob.getLastSensors(t_run)
        rob.updateActuators(update, t_run)

        # Update time and pause until the next time step
        t_real_new = datetime.datetime.now()
        dt_real = (t_real_new - t_real).total_seconds()
        t_run += dt_real
        t_real = t_real_new
        print('t_real = ' + str(t_real) + ' and t_run = ' + str(t_run))
        if dt_real > dt:
            print('Warning: the real time step (' + str(dt_real) + ') is higher than the desired one (' +
                  str(dt) + ')! Please check reading period')
        iteration += 1
        pause.until(t_real)

    # Terminate experiment
    print(str(iteration) + " iterations computed in " +
          str((datetime.datetime.now() - t_real_init).total_seconds()) + " s")
    ctl.plot("cpg.png")
