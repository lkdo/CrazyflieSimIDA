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
import argparse
import math 

# Import local files
import pandaapp
import ftaucf 
import rigidbody
import logger 
import plotter
import envir  
import controllers
import utils
import mems
import spkf
import refs

# Main Simulation Parameters
############################
dt_gps = 1.0/20.0 # GPS meas rate
dt_kf_predict = 1.0/100.0 # KF predict rate
freq_ctrl_rate = 400  # Flight Stab ( and raw mems )
freq_ctrl_angle = 1/dt_kf_predict # Flight Stab
freq_ctrl_pos_v = 10  # Pos Control
freq_ctrl_pos_p = 10  # Pos Control
dt_sim = 1.0/(2*freq_ctrl_rate)  # integration step; has to be bigger than freq_ctrl_rate
dt_log = 0.1  # logging step
dt_vis = 1/30 # visualization frame step
plus = True # Quadrotor configuration, plus or cross 
 

# Initialization values for the quadrotor
##########################################################
pos = np.array([0,0,3]) # position vector in meters 
q = np.array([1,0,0,0]) # unit quaternion  representing attitude 
ve = np.array([0,0,0])  # linear velocity vector in the earth-fixed frame
omegab = np.array([0,0,0]) # angular velocity vector in the body-fixed frame
ab = np.array([0,0,0]) # linear acceleration in body-fixed frame 
qftau = ftaucf.QuadFTau_CF(0,plus) # Model for the forces and torques of the crazyflie (used in simulation)
qftau_s = ftaucf.QuadFTau_CF_S(qftau.cT, qftau.cQ, qftau.radius, 
    qftau.input2omegar_coeff, plus) # Simplified model for the forces and torques (used in control)

# Add some noise to forces and torque model
###########################################
cmd_eq = 37248
fraction = 0.1
sigma_ftau = [ fraction*qftau.input2thrust_i(37248), fraction*qftau.thrust2torque_i(qftau.input2thrust_i(37248)),
    fraction*qftau.input2omegar_i(37248), 0 ]
qftau.sigma = sigma_ftau

qrb = rigidbody.rigidbody(pos, q, ve, omegab, ab, qftau.mass, qftau.I) # Rigid body motion object

# Initialize MEMS sensors 
##########################################################
gyro_x = mems.mems(0.0035/180.0*math.pi,0.000023/180.0*math.pi,0)
gyro_y = mems.mems(0.0035/180.0*math.pi,0.000023/180.0*math.pi,0)
gyro_z = mems.mems(0.0035/180.0*math.pi,0.000023/180.0*math.pi,0)

acc_x = mems.mems(0.140*(10**-3)*9.80665,0.0032*(10**-3)*9.80665,0)
acc_y = mems.mems(0.140*(10**-3)*9.80665,0.0032*(10**-3)*9.80665,0)
acc_z = mems.mems(0.140*(10**-3)*9.80665,0.0032*(10**-3)*9.80665,0)

# averaging buffers for down-sampling
meas_gx_av = 0
meas_gy_av = 0
meas_gz_av = 0

meas_ax_av = 0
meas_ay_av = 0
meas_az_av = 0

# Initialize controller  
##########################################################
att_controller = controllers.AttController_01(freq_ctrl_rate, freq_ctrl_angle)
pos_controller = controllers.PosController_01(freq_ctrl_pos_v, freq_ctrl_pos_p)

omegab_ref = np.zeros(3)
tau_ref = np.zeros(3)
thrust_ref = qrb.mass*envir.g
rpy_ref = np.zeros(3)
pos_ref = np.zeros(3)
ref = np.array([0.0,0.0,3.0,0.0]) #  x,y,z,yaw

# Initialize GPS sensors 
##########################################################
sigma_gps = 0.01
meas_pos = qrb.pos + np.random.normal(0, sigma_gps, 3) # GPS first meas

# Initialize UKF (using Euler Angles)
##########################################################
x0 = np.array([ meas_pos[0], meas_pos[1], meas_pos[2], 0, 0, 0, 0, 0, 0 ]) # x = [pos,euler,ve]
P0 = np.diag([100.0, 100.0, 100.0, 0.01, 0.01, 9.0, 9.0, 9.0, 9.0])
Q = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01])
R = sigma_gps**2*np.diag([1,1,1])

alpha = 1*10**(-3)
kappa = 0
beta = 2.0
meas_func = lambda x: x[0:3] # we measure only position; omegab and ab are used as inputs for the filter, not states
dfx = rigidbody.quadrotor_dt_kinematic_euler

sut = spkf.SUT(alpha, beta, kappa, x0.shape[0])
filter = spkf.SPKF(dfx,meas_func,np.eye(x0.shape[0]),Q,R,x0,P0,sut,variant=1) # IUKF

# Initialize predefined controller references 
##########################################################
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("ref_mode", help=""" Choose between an angle reference
                        template. Values are: step, ramp, sin, manual  """ )
args = arg_parser.parse_args()
if ( args.ref_mode != "manual" ):
    pos_x_ref, pos_y_ref, pos_z_ref, yaw_ref, name2 = refs.buildCtrlReference(args.ref_mode)
else:
    name2 = "manual"

# Initialize the visualization
#########################################################
ctrl_mode = 3 # Full Position Control
name1 = "PosControl"
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

# The main simulation loop     
#########################################################
while readkeys.exitpressed is False :

    #------------------------------------begin sensors -----------------------------------------------

    meas_gx = gyro_x.run_mems(dt_sim,qrb.omegab[0])  # gyro running at simulation freq
    meas_gy = gyro_y.run_mems(dt_sim,qrb.omegab[1])
    meas_gz = gyro_z.run_mems(dt_sim,qrb.omegab[2])

    meas_ax = acc_x.run_mems(dt_sim,qrb.ab[0])  # acc running at simulation freq
    meas_ay = acc_y.run_mems(dt_sim,qrb.ab[1])
    meas_az = acc_z.run_mems(dt_sim,qrb.ab[2])
    
    #-------------------------------------avearge gyroscope meas for kf predict-----------------------
    
    meas_gx_av += meas_gx 
    meas_gy_av += meas_gy
    meas_gz_av += meas_gz

    meas_ax_av += meas_ax
    meas_ay_av += meas_ay
    meas_az_av += meas_az

    #------------------------------------- GPS -----------------------

    if abs(t/dt_gps - round(t/dt_gps)) < 0.000001 :  
        meas_pos = qrb.pos + np.random.normal(0, sigma_gps, 3) # GNSS sensor, simple noise
        filter.update(meas_pos, 1) # Joseph form covariance update 
        filter.x[3:6] = utils.wrap_euler(filter.x[3:6]) # if the innovation has angles, I should wrap those too, but it doesn't 
    
    #------------------------------------begin controller --------------------------------------------

    if abs(t/pos_controller.dt_ctrl_pos_p - round(t/pos_controller.dt_ctrl_pos_p)) < 0.000001 :
        
        # reference
        if ( args.ref_mode == "manual" ):
            pos_ref = readkeys.ref
        else:
            pos_ref[0] = utils.give_signal(pos_x_ref, t)
            pos_ref[1] = utils.give_signal(pos_y_ref, t)
            pos_ref[2] = utils.give_signal(pos_z_ref, t)
            if ( t > max(pos_x_ref[-1,0], pos_y_ref[-1,0], pos_z_ref[-1,0]) ):
                readkeys.exitpressed = True 

        ve_ref = pos_controller.run_pos(pos_ref, meas_pos)
        
    if abs(t/pos_controller.dt_ctrl_pos_v - round(t/pos_controller.dt_ctrl_pos_v)) < 0.000001 :
        
        # use filter estimates 
        est_yaw = filter.x[5]
        est_vel = filter.x[6:9]
        
        rp_ref, thrust_ref = pos_controller.run_vel(ve_ref, est_vel, est_yaw, qrb.mass)
        
        rpy_ref[0] = rp_ref[0]
        rpy_ref[1] = rp_ref[1]
        
    if abs(t/att_controller.dt_ctrl_angle - round(t/att_controller.dt_ctrl_angle)) < 0.000001 :
         
        # use filter estimates
        est_rpy = filter.x[3:6]

        if ( args.ref_mode == "manual" ):
            rpy_ref[2] = readkeys.ref[3]
        else:
            rpy_ref[2] = utils.give_signal(yaw_ref, t)
             
        # controller call 
        omegab_ref = att_controller.run_angle(rpy_ref, est_rpy)
        
    if abs(t/att_controller.dt_ctrl_rate - round(t/att_controller.dt_ctrl_rate)) < 0.000001 :
        
        # Controller call 
        tau_ref = att_controller.run_rate(omegab_ref, np.array([meas_gx,meas_gy,meas_gz]), qrb.I)
        
        # Control allocation; use qftau_s model 
        cmd = qftau_s.fztau2cmd(np.array([thrust_ref,tau_ref[0],tau_ref[1],tau_ref[2]]))
   
    #------------------------------------------ end controller --------------------------------------

    #------------------------------------------ begin filter ----------------------------------------

    if abs(t/dt_kf_predict - round(t/dt_kf_predict)) < 0.000001 :
        
        filter.predict(np.array([meas_ax_av,meas_ay_av,meas_az_av,meas_gx_av,meas_gy_av,meas_gz_av])/
            (dt_kf_predict/dt_sim), dt_kf_predict, 1) # Euler integration
        filter.x[3:7] = utils.wrap_euler(filter.x[3:7])
        
        # Reset down-sampling buffer 
        meas_gx_av = 0
        meas_gy_av = 0
        meas_gz_av = 0

        meas_ax_av = 0
        meas_ay_av = 0
        meas_az_av = 0

        # print(np.diag(filter.P))

    #------------------------------------------ begin simulation -------------------------------------   
    
    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    
    # Run the kinematic / time forward
    qrb.run_quadrotor_dynamic_quat(dt_sim, fb, taub)

    # Time has increased now
    t = t + dt_sim

    # Visualization frequency    
    if abs(t/dt_vis - round(t/dt_vis)) < 0.000001 :
        panda3D_app.taskMgr.step()
        panda3D_app.screenText_pos(qrb.pos,qrb.rpy,filter.x[:3],filter.x[3:6])
        panda3D_app.screenText_ref(np.append(pos_ref,rpy_ref[2]))
        
    # Logging frequency    
    if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
        logger.log_attstab(t,np.array([rpy_ref[0],rpy_ref[1],rpy_ref[2]]),
            np.array([omegab_ref[0],omegab_ref[1],omegab_ref[2]]),
            np.array([tau_ref[0],tau_ref[1],tau_ref[2]]) )
        logger.log_posctrl(t,np.array([ pos_ref[0], pos_ref[1], pos_ref[2] ]))
        logger.log_rigidbody(t, qrb)
        logger.log_filter(t, filter.x)
        fe = qrb.rotmb2e@fb + qrb.mass*np.array([0,0,-envir.g])
        taue = qrb.rotmb2e@taub
        logger.log_ftau(t,fe,taue,fb,taub)
        logger.log_cmd(t, cmd)

        
# End of program, wrap it up with logger and plotter  
#########################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_ftau()
logger.log2file_attstab()
logger.log2file_posctrl()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
plotter.plot_attstab(logger)
plotter.plot_posctrl(logger)
plotter.plot_filter(logger)