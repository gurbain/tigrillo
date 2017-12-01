"""
Script to control the robot over ROS via the NRP. It also send back the results to the platform
"""


from tigrillo.rpi import robot, rosutils
from tigrillo.core.timing import Timer

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "November 25th, 2017"


RESULTS_FOLDER = "../results/run/"


if __name__ == '__main__':

    # Simulation variables
    node_name = "tigrillo_robot"
    act_topic_name = "helloworldsensors"
    sen_topic_name = "helloworldactuators"
    sim_timeout = 100
    sim_dt = 0.1

    # Create and configure robot (by defaut: save actuators and sensors values)
    rob = robot.Tigrillo(data_folder=RESULTS_FOLDER)
    rob.start()
    rob.set_sensor_period(sim_dt/3)

    # Create a ROS node
    node = rosutils.TigrilloROS(node_name=node_name, act_name=act_topic_name, sen_name=sen_topic_name)
    node.start()

    # Create a Timer to ensure that simulation time goes at real-time
    t = Timer(real_time=True, runtime=sim_timeout, dt=sim_dt)
    t.start()

    # Perform experiment
    while not t.is_finished():

        # Get actuation topic value and input in the robot
        update = node.get_last_actuators()
        rob.update_actuators(update, t.st)

        # Update to the robot (to save call both actuators and sensors)
        measure = rob.get_last_sensors(t.st)
        node.update_sensors(measure, t.st)

        # Update time and pause until the next time step
        t.update()

    # Terminate experiment
    t.print_info()
