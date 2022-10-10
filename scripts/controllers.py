# -*- coding: utf-8 -*-
# 
# Copyright 2019 Luminita-Cristiana Totu
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

""" Attitude controller using Euler angles """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import math 
import numpy as np 

import pid 
import utils
import envir

class AttController_01:
    
    def __init__(self, freq_ctrl_rate = 500, freq_ctrl_angle = 250): # 500, 250 Hz
    
        self.dt_ctrl_rate = 1.0/freq_ctrl_rate

        #ToDo0: Tune PIDs 
        self.pid_rollrate = pid.PID(50, 0, 0, 8*360*math.pi/180,-8*360*math.pi/180, 0.01)
        self.pid_pitchrate = pid.PID(50, 0, 0, 8*360*math.pi/180,-8*360*math.pi/180, 0.01)
        self.pid_yawrate = pid.PID(50, 0, 0, 8*360*math.pi/180, -8*360*math.pi/180, 0.01)

        self.dt_ctrl_angle = 1.0/freq_ctrl_angle  
        
        #ToDo0: Tune PIDs 
        self.pid_pitch = pid.PID(4, 0, 0, 2.5*360*math.pi/180, -2.5*360*math.pi/180, 0.01)
        self.pid_roll = pid.PID(4, 0, 0, 2.5*360*math.pi/180, -2.5*360*math.pi/180, 0.01)
        self.pid_yaw = pid.PID(3, 0, 0, 2.5*360*math.pi/180, -2.5*360**math.pi/180, 0.01)

    def run_rate(self, ref_omegab, meas_omegab, J ):
        
        alpha_ref = np. zeros(3)
        
        alpha_ref[0] = self.pid_rollrate.run(ref_omegab[0]-meas_omegab[0], self.dt_ctrl_rate)
        alpha_ref[0] = self.pid_rollrate.saturate()
        self.pid_rollrate.antiwindup()
        
        alpha_ref[1] = self.pid_pitchrate.run(ref_omegab[1]-meas_omegab[1],self.dt_ctrl_rate)
        alpha_ref[1] = self.pid_pitchrate.saturate()
        self.pid_pitchrate.antiwindup()
        
        alpha_ref[2] = self.pid_yawrate.run(ref_omegab[2]-meas_omegab[2],self.dt_ctrl_rate)
        alpha_ref[2] = self.pid_yawrate.saturate()
        self.pid_yawrate.antiwindup()
        
        # Inverse kinematics
        tau_ref = J@alpha_ref + utils.skew(meas_omegab)@J@meas_omegab

        return tau_ref

    def run_angle(self, ref_rpy, meas_rpy ):
        
        omega_ref = np. zeros(3)
        
        omega_ref[0] = self.pid_roll.run(ref_rpy[0]-meas_rpy[0],self.dt_ctrl_angle)
        omega_ref[0] = self.pid_roll.saturate()
        self.pid_roll.antiwindup()
        
        omega_ref[1] = self.pid_pitch.run(ref_rpy[1]-meas_rpy[1],self.dt_ctrl_angle)
        omega_ref[1] = self.pid_pitch.saturate()
        self.pid_pitch.antiwindup()
        
       # make possible 2*pi degrees yaw movement 
       # by bridging the discontinuity -pi + pi 
        err_yaw = ref_rpy[2]-meas_rpy[2]
        if (err_yaw > math.pi):
            err_yaw = -(2*math.pi - err_yaw)
        elif (err_yaw < -math.pi):
            err_yaw = 2*math.pi + err_yaw
        
        omega_ref[2] = self.pid_yaw.run(err_yaw,self.dt_ctrl_angle)
        omega_ref[2] = self.pid_yaw.saturate()
        self.pid_yaw.antiwindup()
        
        return omega_ref
    

class PosController_01:

    def __init__(self, freq_ctrl_pos_v = 10, freq_ctrl_pos_p = 10 ): 
        
        self.dt_ctrl_pos_v = 1.0/freq_ctrl_pos_v
          
        #ToDo2: Tune PIDs 
        # saturation in the directional  way 
        self.pid_vx = pid.PID(1, 0, 0, 9999, -9999, 0.1)
        self.pid_vy = pid.PID(1, 0, 0, 9999, -9999, 0.1)
        self.pid_vz = pid.PID(1, 0, 0, 9999, -9999, 0.1)

        #ToDo2: Tune PIDs
        self.dt_ctrl_pos_p = 1.0/freq_ctrl_pos_p
        self.pid_x = pid.PID(1, 0, 0, 20, -20, 0.1)
        self.pid_y = pid.PID(1, 0, 0, 20, -20, 0.1)
        self.pid_z = pid.PID(1, 0, 3.5, 10, -10, 0.1)

        self.max_thrust =  0.8*0.638
    
    def run_vel(self, ref_ve, meas_ve, meas_yaw, mass):
        
        rp_ref = np.zeros(2)
        
        T1 = self.pid_vx.run(ref_ve[0]-meas_ve[0],self.dt_ctrl_pos_v)
        T2 = self.pid_vy.run(ref_ve[1]-meas_ve[1],self.dt_ctrl_pos_v)
        
        R = np.array([[math.sin(meas_yaw), -math.cos(meas_yaw)],
                              [math.cos(meas_yaw), math.sin(meas_yaw)]])
        
        rp_ref = 1/envir.g *R@np.array([T1,T2])
        
        # And saturate 
        V = 30 * math.pi /180 
        v_max = np.max(abs(rp_ref))
        if (v_max > V):
            rp_ref = (V/v_max)*rp_ref
        
        # recalculate to do antiwindup 
        T12 = np.transpose(R)*envir.g*rp_ref
        self.pid_vx.u = T12[0]
        self.pid_vx.antiwindup()
        self.pid_vy.u = T12[1]
        self.pid_vy.antiwindup()
        
        T3 = self.pid_vz.run(ref_ve[2] - meas_ve[2], self.dt_ctrl_pos_v)
        thrust_ref = mass*envir.g + mass*T3 
 
        # And saturate 
        if ( thrust_ref > self.max_thrust ):
            thrust_ref = self.max_thrust
        elif (thrust_ref < 0.8*mass*envir.g ):
            thrust_ref =  0.8*mass*envir.g 
        
        # recalculate to do anti-windup
        T3 = (thrust_ref - mass*envir.g)/mass
        self.pid_vz.u = T3
        self.pid_vz.antiwindup()
        
        return rp_ref, thrust_ref
    
    def run_pos(self, ref_pos, meas_pos):
        
        ref_ve = np.zeros(3)
        
        ref_ve[0] = self.pid_x.run(ref_pos[0] - meas_pos[0], self.dt_ctrl_pos_p)
        ref_ve[1] = self.pid_y.run(ref_pos[1] - meas_pos[1], self.dt_ctrl_pos_p)
        ref_ve[2] = self.pid_z.run(ref_pos[2] - meas_pos[2], self.dt_ctrl_pos_p)
        
        return ref_ve