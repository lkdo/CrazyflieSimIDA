# -*- coding: utf-8 -*-
# 
# Copyright 2021 Luminita-Cristiana Totu
#
# Part of the QuadrotorSim aka quadsim package
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this package, in a file called LICENSE.
# If not, see <https://www.gnu.org/licenses/>.

""" Main loop of simulation for Rigid Body Model with Forces and Torques """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

# Import general libraries
import numpy as np
import time
import datetime
import math 

# Import local files
import pandaapp
import ftaucf 
import rigidbody
import logger 
import plotter
import envir  
import controllers
import mems

# Main Simulation Parameters
############################
dt_sim = 1/800.0  # integration step; has to be bigger than freq_ctrl_rate
dt_log = 0.1  # logging step
dt_vis = 1/30 # visualization frame step
plus = True # Quadrotor configuration, plus or cross 


# Initialization values for the quadrotor
##########################################################
pos = np.array([0,0,3]) # position vector in meters 
q = np.array([1,0,0,0]) # unit quaternion representing attitude 
ve = np.array([0,0,0])  # linear velocity vector in the earth-fixed frame
omegab = np.array([0,0,0]) # angular velocity vector in the body-fixed frame
ab = np.array([0,0,0]) # linear acceleration in body-fixed frame 
qftau = ftaucf.QuadFTau_CF(0,plus) # Model for the forces and torques of the crazyflie (used in simulation)
qftau_s = ftaucf.QuadFTau_CF_S(qftau.cT, qftau.cQ, qftau.radius, 
    qftau.input2omegar_coeff, plus) # Simplified model for the forces and torques (used in control)
qrb = rigidbody.rigidbody(pos, q, ve, omegab, ab, qftau.mass, qftau.I) # Rigid body motion object

# Initialize predefined controller references 
##########################################################

# Initialize the visualization
#########################################################
ctrl_mode = 0
name2 = "manual"
name1 = "manual"
ref = qftau_s.fztau2cmd(np.array([qrb.mass*envir.g, 0, 0, 0]))
print(ref)
panda3D_app = pandaapp.Panda3DApp(plus,qrb,ref,ctrl_mode)
readkeys = pandaapp.ReadKeys(ref,ctrl_mode,panda3D_app)

# Initialize the logger & plotter 
##########################################################
ts = time.time()
name = name1 + "_" + name2 +datetime.datetime.fromtimestamp(ts).strftime("_%Y%m%d%H%M%S")
""" Name of run to save to log files and plots """
fullname = "logs/" + name
logger = logger.Logger(fullname, name)
plotter = plotter.Plotter()

# Initialize others
#########################################################
t = 0
cmd = np.array([readkeys.ref[0], readkeys.ref[1], readkeys.ref[2], readkeys.ref[3]])


# The main simulation loop     
#########################################################
while readkeys.exitpressed is False :

    #------------------------------------------ begin simulation -------------------------------------   

    if abs(t/0.01 - round(t/0.01)) < 0.000001 :
        cmd = np.array([readkeys.ref[0], readkeys.ref[1], readkeys.ref[2], readkeys.ref[3]])

    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    
    # Run the kinematic / time forward
    qrb.run_quadrotor_dynamic_quat(dt_sim, fb, taub)

    # Time has increased now
    t = t + dt_sim

    # Visualization frequency    
    if abs(t/dt_vis - round(t/dt_vis)) < 0.000001 :
        panda3D_app.taskMgr.step()
        panda3D_app.screenText_pos(qrb.pos,qrb.rpy,np.zeros(3),np.zeros(3))
        panda3D_app.screenText_ref(np.array(cmd))
        
    # Logging frequency    
    if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
        logger.log_rigidbody(t, qrb)
        fe = qrb.rotmb2e@fb + qrb.mass*np.array([0,0,-envir.g])
        taue = qrb.rotmb2e@taub
        logger.log_ftau(t,fe,taue,fb,taub)
        logger.log_cmd(t, cmd)


# End of program, wrap it up with logger and plotter  
#########################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_ftau()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
