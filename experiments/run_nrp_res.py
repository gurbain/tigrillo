"""
Script to perform open loop run using hand-made splined with the Tigrillo Robot and save all sensors data
"""


from tigrillo.rpi import robot
from tigrillo.core.control import CPG
from tigrillo.core.timing import Timer

import numpy as np

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 12th, 2017" 


RES_FOLDER = "../results/run/"
NRP_RES_FOLDER = "../results/nrp/"


if __name__ == '__main__':

    # Create and configure robot (by defaut: save actuators and sensors values)
    rob = robot.Tigrillo(data_folder=RES_FOLDER)
    rob.start()
    rob.set_sensor_period(0.1)

    # Load a config file from a NRP result
    cpg_params = np.load(NRP_RES_FOLDER + "2017-9-20-23-36-4_tigrillo_scaled_feets_D7N30_it86.npy")
    for a in cpg_params:
        a["omega"] = a["omega"] * 2
    config = {"Controller": {"params": cpg_params, "integ_time": 0.001, "timestep": 0.02, "runtime": 40}}
    ctl = CPG(config)

    # Create a Timer to ensure that simulation time goes at real-time
    t = Timer(real_time=True, runtime=config["Controller"]["runtime"], dt=config["Controller"]["timestep"])
    t.start()

    # Perform experiment
    while not t.is_finished():

        # Produce actuators control signal for open loop
        cmd = ctl.step(t.st)

        # Translate command representation (offset of 115 degrees)
        update = {"FL": cmd[0] + 155, "FR": cmd[1] + 155,
                  "BL": cmd[2] + 155, "BR": cmd[3] + 155}

        # Update to the robot (to save call both actuators and sensors)
        measure = rob.get_last_sensors(t.st)
        rob.update_actuators(update, t.st)

        # Update time and pause until the next time step
        t.update()

    # Terminate experiment
    t.print_info()
    ctl.plot("cpg.png")
