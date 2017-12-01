"""
This file provides a wrapper to run a physics simulator. In the current
version, only gazebo is working, but I try to extend it to pybullet as well
as Bullet in C++ to be able to run fast simulations that can also simulate
deformable objects and with python handles
"""

import numpy as np
import psutil
# import pybullet as p
import rospy as ros
import subprocess

from tigrillo.core.utils import *

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, Float64
from std_srvs.srv import Empty

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 14th, 2017"


class Physics(object):

    def __init__(self, config):

        self.config = config
        self.log = logging.getLogger('Physics')

        self.sim_pid = None
        self.sim_status = None

        self.sim_duration = 0
        self.sim_model_state = {"pos": {"x": 0, "y": 0, "z": 0}, "ang": {"x": 0, "y": 0, "z": 0, "w": 0}}

        return

    def start_sim(self):

        return

    def stop_sim(self):

        return

    def pause_sim(self):

        return

    def unpause_sim(self):

        return

    def reset_sim(self):

        return


class Bullet(Physics):
    """
    This class can use the pybullet libraries to run the physics of
    the simulation. In opposition to Gazebo, it runs synchronously.
    IT IS CURRENTLY BROKEN!!!
    """

    def __init__(self, config):

        super(Physics, self).__init__(config)

        self.t_step = 0.001
        self.plane = "models/plane.urdf"

        self.t = 0
        self.t_max = 7
        self.t_start = None
        self.t_stop = None

        self.dim = int(1./self.t_step) * self.t_max + 1
        self.cont = None  # controller
        self.rt = None  # rt
        self.mode = None  # mode
        self.robust = None  # robust
        self.model = None  # model
        self.robot = None
        self.n_joint = 0
        self.pos_start = None
        self.ori_start = None
        self.noise = None

        self.imp_av_inter_time = 1.5
        self.imp_time = 0.1
        self.noise_max = 3  # N (the robot weight 1kg so 10N are enough to lift it)

        return

    def start_sim(self):

        self.init_sim()
        self.run_sim()

    def init_sim(self):

        # Init simulator
        if self.mode == "gui":
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.loadURDF(self.plane, 0, 0, -1.5)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.t_step)
        self.robot = p.loadSDF(self.model)[0]

        # Create noise vector
        if self.robust == 1:
            self.create_noise()

        # Reset pose
        self.n_joint = p.getNumJoints(self.robot)
        j = 0
        for i in range(self.n_joint):
            name = p.getJointInfo(self.robot, i)[1].decode('UTF-8')
            if name.find("UJ") != -1:
                p.resetJointState(self.robot, i, self.cont.step(self.t, j, 0, 0))
                j += 1
            else:
                # Here we add a spring
                p.resetJointState(self.robot, i, 0)

        # Get initial states
        self.pos_start, self.ori_start = p.getBasePositionAndOrientation(self.robot)

        return

    def run_sim(self):

        self.t_start = time.time()
        i = 0
        while self.t < self.t_max:
            self.t = self.t + self.t_step
            self.step_sim(i)
            i += 1
        self.t_stop = time.time()

    def stop_sim(self):

        p.disconnect()

        return

    def step_sim(self, step):

        time_start_it = time.time()
        j = 0
        for i in range(self.n_joint):
            name = p.getJointInfo(self.robot, i)[1].decode('UTF-8')
            if name.find("UJ") != -1:
                # pos_feed, speed_feed, r_f, t = p.getJointState(self.robot, i)
                p.setJointMotorControl2(self.robot, i, p.POSITION_CONTROL,
                                        targetPosition=self.cont.step(self.t, j, 0, 0))
                j += 1

        # Apply noise
        if self.robust == 1 and self.noise[step][0] != 0:
            p.applyExternalForce(self.robot, -1, self.noise[step].tolist(),
                                 [0, 0, 0], p.LINK_FRAME)

        # Run physics step
        if self.rt:
            while time.time() < time_start_it + 0.85 * self.t_step:
                time.sleep(self.t_step/100)
        p.stepSimulation()

    def get_results(self):

        # print(p.getContactPoints(self.robot))
        pos, ori = p.getBasePositionAndOrientation(self.robot)
        x_dist = (pos[1] - self.pos_start[1]) - np.abs(pos[0] - self.pos_start[0])/10

        return x_dist

    def create_noise(self):

        self.noise = np.zeros((self.dim, 3))
        n_imp = int(self.t_max / self.imp_av_inter_time)
        t_start = np.random.randint(0, self.dim, int(n_imp))
        t_imp = []
        for i in range(int(self.imp_time / self.t_step)):
            t_imp.extend(np.add(t_start, i).tolist())
        first = True
        for i in range(self.dim):
            if i in t_imp:
                if first:
                    val = np.random.uniform(-self.noise_max, self.noise_max, 3)
                    first = False
                    self.noise[i] += val
            else:
                first = True


class Gazebo(Physics):
    """
    This class can start a gazebo simulation process and provides
    the handles to control it. It also enables to start and stop the
    roscore as a separate process, though it is advised to do it in a
    separate bash script to avoid concurrent runs, the lons starting
    phase and to close it correctly.
    """

    def __init__(self, config):

        super(Gazebo, self).__init__(config)

        self.config = config
        self.log = logging.getLogger('Gazebo')

        self.sim_ps = None
        self.sim_ps_name = "rosrun"
        self.sim_package = "gazebo_ros"
        if config["Physics"]["rendering"] == "True":
            self.log.info("Using gazebo with rendering")
            self.sim_node = "gazebo"
        else:
            self.sim_node = "gzserver"
        self.sim_args = config["Physics"]["model"]
        self.gzserver_name = 'gzserver'
        self.gzclient_name = 'gzclient'

        self.reset_sim_service = '/gazebo/reset_simulation'
        self.pause_sim_service = '/gazebo/pause_physics'
        self.unpause_sim_service = '/gazebo/unpause_physics'
        self.reset_sim_proxy = ros.ServiceProxy(self.reset_sim_service, Empty)
        self.pause_sim_proxy = ros.ServiceProxy(self.pause_sim_service, Empty)
        self.unpause_sim_proxy = ros.ServiceProxy(self.unpause_sim_service, Empty)

        self.ros_pid = None
        self.ros_status = None
        self.ros_ps = None
        self.ros_ps_name = "roscore"
        self.rosmaster_ps_name = "rosmaster"

        self.sub_clock = None
        self.sub_state = None
        self.pub_legs = None
        self.pub_leg_fl = None
        self.pub_leg_fr = None
        self.pub_leg_bl = None
        self.pub_leg_br = None

    def start_sim(self):

        self.start_roscore()
        self.start_gazebo()
        ros.init_node('physics', anonymous=True)
        self.log.info("Initializing sensors and actuators subscribers and publishers")

        # Create Subscribers
        self.sub_clock = ros.Subscriber("/clock", Clock,
                                        callback=self._reg_sim_duration, queue_size=1)
        self.sub_state = ros.Subscriber("/gazebo/model_states", ModelStates,
                                        callback=self._reg_sim_states, queue_size=1)

        # Create Publishers (TO CHANGE)
        self.pub_leg_fl = ros.Publisher('/tigrillo/tigrillo_feet__left_shoulder/cmd_pos', Float64, queue_size=1)
        self.pub_leg_fr = ros.Publisher('/tigrillo/tigrillo_feet__right_shoulder/cmd_pos', Float64, queue_size=1)
        self.pub_leg_bl = ros.Publisher('/tigrillo/tigrillo_feet__left_hip/cmd_pos', Float64, queue_size=1)
        self.pub_leg_br = ros.Publisher('/tigrillo/tigrillo_feet__right_hip/cmd_pos', Float64, queue_size=1)
        # self.pub_legs = ros.Publisher('/tigrillo/legs_cmd', Float32MultiArray, queue_size=1)

    def stop_sim(self):

        self.stop_gazebo()

    def start_gazebo(self):

        self.sim_duration = 0

        self.get_gazebo_status()
        if self.sim_status == psutil.STATUS_RUNNING or self.sim_status == psutil.STATUS_SLEEPING:
            self.log.warning("Simulation is already started")
            return

        self.log.warning("Starting GAZEBO ROS node")
        proc = [self.sim_ps_name, self.sim_package, self.sim_node, self.sim_args]
        self.sim_ps = subprocess.Popen(proc, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
        proc_out, proc_err = self.sim_ps.communicate()
        self.log.info(proc_out)
        self.log.warning("Starting Gazebo with params: " + str(proc))

    def stop_gazebo(self):

        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count(self.gzclient_name)
        gzserver_count = tmp.count(self.gzserver_name)

        if gzclient_count > 0:
            os.system("killall -9 " + self.gzclient_name)
        if gzserver_count > 0:
            os.system("killall -9 " + self.gzserver_name)

        if gzclient_count or gzserver_count > 0:
            os.wait()

    def reset_gazebo(self):

        ros.wait_for_service(self.reset_sim_service)
        try:
            self.reset_sim_proxy()
        except ros.ServiceException as e:
            self.log.error("Reset simulation service call failed with error" + str(e))

    def pause_gazebo(self):

        ros.wait_for_service(self.pause_sim_service)
        try:
            self.pause_sim_proxy()
        except ros.ServiceException as e:
            self.log.error("Pause simulation service call failed with error" + str(e))

    def unpause_gazebo(self):

        ros.wait_for_service(self.unpause_sim_service)
        try:
            self.unpause_sim_proxy()
        except ros.ServiceException as e:
            self.log.error("Unpause simulation service call failed with error" + str(e))

    def get_gazebo_status(self):

        self.sim_status = psutil.STATUS_STOPPED

        for proc in psutil.process_iter():
            if proc.name() == self.sim_ps_name or proc.name() == self.sim_node:
                self.sim_status = proc.status()
                self.sim_pid = proc.pid

        return self.sim_status

    def start_roscore(self):

        self.get_roscore_status()
        if self.ros_status == psutil.STATUS_RUNNING or self.ros_status == psutil.STATUS_SLEEPING:
            self.log.warning("ROS is already started")
            return

        self.ros_ps = subprocess.Popen([self.ros_ps_name], shell=False)

        return

    def stop_roscore(self):

        tmp = os.popen("ps -Af").read()
        roscore_count = tmp.count(self.ros_ps_name)
        rosmaster_count = tmp.count(self.rosmaster_ps_name)

        if rosmaster_count > 0:
            os.system("killall -9 " + self.ros_ps_name)
        if roscore_count > 0:
            os.system("killall -9 " + self.rosmaster_ps_name)

        if roscore_count or rosmaster_count > 0:
            os.wait()

    def get_roscore_status(self):

        self.ros_status = psutil.STATUS_STOPPED

        for proc in psutil.process_iter():
            if proc.name() == self.ros_ps_name:
                self.ros_status = proc.status()
                self.ros_pid = proc.pid

        return self.ros_status

    def _reg_sim_duration(self, time):

        self.sim_duration = time.clock.secs + time.clock.nsecs/1000000000.0

    def _reg_sim_states(self, states):

        index = -1
        for i, name in enumerate(states.name):
            if name == "tigrillo":
                index = i

        if index != -1:
            self.sim_model_state["pos"]["x"] = states.pose[index].position.x
            self.sim_model_state["pos"]["y"] = states.pose[index].position.y
            self.sim_model_state["pos"]["z"] = states.pose[index].position.z
            self.sim_model_state["ang"]["x"] = states.pose[index].orientation.x
            self.sim_model_state["ang"]["y"] = states.pose[index].orientation.y
            self.sim_model_state["ang"]["z"] = states.pose[index].orientation.z
            self.sim_model_state["ang"]["w"] = states.pose[index].orientation.w

        # print(self.sim_model_state)
        return

    def is_sim_started(self):

        return self.sim_duration > 0

    def set_sim_cmd(self, cmd):

        # TO CHANGE
        if len(cmd) == 4:
            self.pub_leg_fl.publish(Float64(cmd[0]))
            self.pub_leg_fr.publish(Float64(cmd[1]))
            self.pub_leg_bl.publish(Float64(cmd[2]))
            self.pub_leg_br.publish(Float64(cmd[3]))
        # self.pub_legs.publish(Float32MultiArray(layout=MultiArrayLayout([], 1), data=cmd))
        return


if __name__ == '__main__':

    # Test Gazebo sim
    config = set_default_logger()
    log = logging.getLogger('Main')

    log.debug("ici")
    log.info("la")
    log.warning("br")
    log.error("er")

    config["Physics"] = {"rendering": "True", "model": "/home/gabs48/tigrillo/data/worlds/tigrillo_feet.world"}
    p = Gazebo(config)
    print (config.__dict__)

    # log.warning("[ " + str(time.time()) + " ] Init ROS")
    # p.start_roscore()

    for i in range(2):
        log.warning("Start sim " + str(i))
        print("a")
        p.start_sim()
        time.sleep(10)
        p.stop_sim()
        log.debug("ici")
        log.info("la")
        log.warning("br")
        log.error("er")

    # log.warning("[ " + str(time.time()) + " ] Stop ROS")
    # p.stop_roscore()
