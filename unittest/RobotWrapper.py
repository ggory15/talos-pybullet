import pinocchio
from example_robot_data import loadTalos # Functions to load the Talos


import gepetto.corbaserver
import numpy as np

import subprocess, os, time

# Load robot
robot = loadTalos()
robot.initDisplay(loadModel=True)
rmodel = robot.model

# Open Gepetto Viewer
l = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
if int(l[1]) == 0:
    os.system('gepetto-gui &')
time.sleep(1)

cl = gepetto.corbaserver.Client()
robot.initViewer(windowName="Talos", loadModel=True)

# Defining the initial state of the robot
q0 = robot.model.referenceConfigurations['half_sitting'].copy()
v0 = pinocchio.utils.zero(robot.model.nv)
q0[38] = 2.0
robot.display(q0)
