import numpy as np
# Pinocchio modules
import pinocchio as se3  # Pinocchio library
import eigenpy
eigenpy.switchToNumpyMatrix()

import copy
from .PD import PD

################
#  CONTROLLER ##
################
def c_control(q, qdot, dt, robot, t_simu):
    eigenpy.switchToNumpyMatrix()

    # Get dynamics information
    M = robot.mass(q) # Mass Matrix
    NLE = robot.nle(q, qdot) #gravity and Coriolis

    # Torque from Non linear compensation term
    torque = NLE    

    # For our controller will be here
    return torque[6:]

