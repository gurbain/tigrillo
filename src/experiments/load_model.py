import pybullet as p
from time import sleep
import math

# Options
ENABLE_MENU_JOINT_CONTROL = True

# Mode
physicsClient = p.connect(p.GUI)

# Parametrize
timestep = 0.01
t = 0
p.setTimeStep(timestep)
p.setGravity(0,0,-9.81)

# Load models
folder = "../../models/"
plane_pos = [0,0,-2]
plane_ang = p.getQuaternionFromEuler([0,0,0])
plane = p.loadURDF(folder + "plane.urdf", plane_pos, plane_ang)

robot_pos = [0,0,0]
robot_ang = p.getQuaternionFromEuler([0,0,0])
robot = p.loadSDF(folder + "tigrillo.sdf")
robot_id = robot[0]

# Reset simulation
p.resetBasePositionAndOrientation(robot_id, robot_pos, robot_ang)

#robot = p.loadMJCF(folder + "tigrillo.mjcf")

# Add debug
joint_name2joint_index = {}
parameter_id2joint_index = {}
for joint_nr in range(p.getNumJoints(robot_id)):
	joint_info = p.getJointInfo(robot_id, joint_nr)
	joint_idx = joint_info[0]
	joint_name = str(joint_info[1])
	joint_name2joint_index[joint_name] = joint_idx


if ENABLE_MENU_JOINT_CONTROL:
	for joint_name, joint_idx in joint_name2joint_index.items():
		if joint_name.find("D") != -1:
			id = p.addUserDebugParameter(joint_name + "_control", -1, 1, 0)
			parameter_id2joint_index[id] = joint_idx

# Run simulation
while t < 300:
	if ENABLE_MENU_JOINT_CONTROL:
		for parameter_id, joint_index in parameter_id2joint_index.items():
			param_value = p.readUserDebugParameter(parameter_id)
			p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint_index, controlMode=p.VELOCITY_CONTROL,
				targetVelocity=param_value, force=100)
	p.stepSimulation()
	sleep(timestep)
	#print("t = " + str(t))
	t += timestep


