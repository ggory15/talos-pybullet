# coding: utf8

import numpy as np  # Numpy library

import pybullet as p  # PyBullet simulator
import pybullet_data

import gepetto.corbaserver
from example_robot_data import loadTalos # Functions to load the Talos#
import pinocchio

def configure_simulation(dt, enableGUI):
    global jointTorques
    # Load robot
    robot = loadTalos()
    robot.initDisplay(loadModel=True)
    rmodel = robot.model

    # Defining the initial state of the robot
    q0 = robot.model.referenceConfigurations['half_sitting'].copy()
    q0[2] = 1.2 # Note that pinocchio's index is as follow: left leg (7), right leg, torso(2), left arm(8), right arm, head(2)

    v0 = pinocchio.utils.zero(robot.model.nv)
    nq = robot.model.nq
    nv = robot.model.nv
    na = nq - 7 # actual activated joint

    # Start the client for PyBullet
    if enableGUI:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)  # noqa
    # p.GUI for graphical version
    # p.DIRECT for non-graphical version

    # Set gravity (disabled by default)
    p.setGravity(0, 0, -9.81)

    # Load horizontal plane for PyBullet
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Load the robot for PyBullet
    robotStartPos = q0[:3]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath("/opt/openrobots/share/example-robot-data/robots/talos_data/robots")
    robotId = p.loadURDF("talos_reduced.urdf", robotStartPos, robotStartOrientation)
    
    # Set time step of the simulation
    # dt = 0.001
    p.setTimeStep(dt)
    # realTimeSimulation = True # If True then we will sleep in the main loop to have a frequency of 1/dt

    # set Initial Posture
    revoluteJointIndices =  np.concatenate((np.arange(45, 51), np.arange(52, 58), np.arange(0, 2), np.arange(11, 19), np.arange(28, 36), np.arange(3, 5)), axis=None)
    for m in range(na):
        p.resetJointState(robotId, revoluteJointIndices[m], q0[m+7]) 

    # Disable default motor control for revolute joints
    p.setJointMotorControlArray(robotId,
                                jointIndices=revoluteJointIndices,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=[0.0 for m in revoluteJointIndices],
                                forces=[0.0 for m in revoluteJointIndices])

    # Enable torque control for revolute joints
    jointTorques = [0.0 for m in revoluteJointIndices]
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation for initialization
    p.stepSimulation()

    return robotId, robot, revoluteJointIndices


# Function to get the position/velocity of the base and the angular position/velocity of all joints
def getPosVelJoints(robotId, revoluteJointIndices):
    jointStates = p.getJointStates(robotId, revoluteJointIndices)  # State of all joints
    baseState = p.getBasePositionAndOrientation(robotId)  # Position of the free flying base
    baseVel = p.getBaseVelocity(robotId)  # Velocity of the free flying base

    # Reshaping data into q and qdot
    q = np.vstack((np.array([baseState[0]]).transpose(), np.array([baseState[1]]).transpose(),
                   np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
    qdot = np.vstack((np.array([baseVel[0]]).transpose(), np.array([baseVel[1]]).transpose(),
                      np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()))

    return q, qdot

