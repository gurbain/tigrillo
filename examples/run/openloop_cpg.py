"""
Script to perform open loop run using hand-made splined with the Tigrillo Robot and save all sensors data
"""


from tigrillo.rpi import robot
from tigrillo.core.control import CPG
from tigrillo.core.timing import Timer

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

    # Create and configure CPG controller
    config_run = {"Controller":
                  {"params": "[{'mu': 500, 'o': -10, 'omega': 9, 'duty_factor': 0.5, "
                             "'phase_offset': 0, 'coupling': [5,5,5,0]},"
                             "{'mu': 500, 'o': -10, 'omega': 9, 'duty_factor': 0.5, "
                             "'phase_offset': 6.28, 'coupling': [5,5,5,0]},"
                             "{'mu': 1000, 'o': -30, 'omega': 9, 'duty_factor': 0.95, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]},"
                             "{'mu': 1000, 'o': -30, 'omega': 9, 'duty_factor': 0.95, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]}]",
                   "integ_time": 0.001,
                   "timestep": 0.02,
                   "runtime": 40}}
    config_walk = {"Controller":
                  {"params": "[{'mu': 1100, 'o': -15, 'omega': 6, 'duty_factor': 0.9, "
                             "'phase_offset': 0, 'coupling': [5,5,5,0]},"
                             "{'mu': 1100, 'o': -15, 'omega': 6, 'duty_factor': 0.9, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]},"
                             "{'mu': 800, 'o': -40, 'omega': 6, 'duty_factor': 0.9, "
                             "'phase_offset': 3.14, 'coupling': [5,5,5,0]},"
                             "{'mu': 800, 'o': -40, 'omega': 6, 'duty_factor': 0.9, "
                             "'phase_offset': 6.28, 'coupling': [5,5,5,0]}]",
                       "integ_time": 0.001,
                       "timestep": 0.02,
                       "runtime": 40}}
    config = config_run
    ctl = CPG(config)

    # Create a Timer to ensure that simulation time goes at real-time
    t = Timer(real_time=True, runtime=config["Controller"]["runtime"], dt=config["Controller"]["timestep"])
    t.start()

    # Perform experiment
    while not t.is_finished():

        # Produce actuators control signal for open loop
        cmd = ctl.step(t.st)

        # Translate command representation (115 degrees between model and real zero)
        update = {"FL": cmd[0] + 115, "FR": cmd[1] + 115,
                  "BL": cmd[2] + 115, "BR": cmd[3] + 115}

        # Update to the robot (to save call both actuators and sensors)
        measure = rob.get_last_sensors(t.st)
        rob.update_actuators(update, t.st)

        # Update time and pause until the next time step
        t.update()

    # Terminate experiment
    t.print_info()
    ctl.plot("cpg.png")
