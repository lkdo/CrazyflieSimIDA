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

""" This module implements the Graphical Plotter """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import matplotlib.pyplot as plt
import os 

class Plotter:
    """ Handles the plotting from a logger object  """
  
    def __init__(self):
        
        self.big_font_size = 25
        
    def fig_style_1(self, fig, ax_lst):
        
        fig.set_size_inches(20, 15)
        fig.set_dpi(100)
        fig.subplots_adjust(hspace=0.5)
        for ax in ax_lst:
            ax.tick_params(axis='both', which='major', labelsize=20)
            ax.tick_params(axis='both', which='minor', labelsize=10)
            for item in [ax.title, ax.xaxis.label, ax.yaxis.label]:
                     item.set_fontsize(self.big_font_size)
            ax.grid(True)    
        
    def plot_rigidbody(self, log):
        
        # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)
        
        # Position plot
        ################################################
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Rigid Body Position', fontsize = self.big_font_size)
        time = np.asarray(log.rb_time)
        pos = np.asarray(log.rb_pos)
        
        ax_lst[0].plot( time, pos[:,0], marker = "o" )
        ax_lst[0].set_title("X-axis")
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel('Distance [m]')
        ax_lst[1].plot( time, pos[:,1], marker = "o" )
        ax_lst[1].set_title("Y-axis")
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel('Distance [m]')
        ax_lst[2].plot( time, pos[:,2], marker = "o" ) 
        ax_lst[2].set_title("Z-axis")
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel('Distance [m]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__rigidbody_pos.png"
        fig.savefig(fullname)
        plt.close(fig)
        
        # Euler Angles plot 
        ################################################
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Rigid Body Euler-XYZ', fontsize = self.big_font_size)
        time = np.asarray(log.rb_time)
        euler = np.asarray(log.rb_euler)
        
        ax_lst[0].plot( time, euler[:,0], marker = "o" )
        ax_lst[0].set_title("Roll - rotation around the X-axis")
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel('Angles [$^\circ$]')
        ax_lst[1].plot( time, euler[:,1], marker = "o" )
        ax_lst[1].set_title("Pitch - rotation around the Y-axis")
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel('Angles [$^\circ$]')
        ax_lst[2].plot( time, euler[:,2], marker = "o" ) 
        ax_lst[2].set_title("Yaw - rotation around the Z-axis")
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel('Angles [$^\circ$]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename+ "__rigidbody_eulerXYZ.png"
        fig.savefig(fullname)
        plt.close(fig)
        
    def plot_cmd(self, log):
        
        # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)

        # Rotor command plot
        fig, ax_lst = plt.subplots(4, 1)
        fig.suptitle('Raw Rotor Command', fontsize = self.big_font_size)
        time = np.asarray(log.cmd_time)
        cmd_rotors = np.asarray(log.cmd_rotors)
        
        ax_lst[0].plot( time, cmd_rotors[:,0], marker = "o" )
        ax_lst[0].set_title("Rotor 1")
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel('16-bit number')
        
        ax_lst[1].plot( time, cmd_rotors[:,1], marker = "o" )
        ax_lst[1].set_title("Rotor 2")
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel('16-bit number')
        
        ax_lst[2].plot( time, cmd_rotors[:,2], marker = "o" ) 
        ax_lst[2].set_title("Rotor 3")
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel('16-bit number')
        
        ax_lst[3].plot( time, cmd_rotors[:,3], marker = "o" ) 
        ax_lst[3].set_title("Rotor 4")
        ax_lst[3].set_xlabel('Time [s]');ax_lst[3].set_ylabel('16-bit number')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__cmd_rotors.png"
        fig.savefig(fullname)
        plt.close(fig)
        
    def plot_attstab(self, log):        
        
        # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)

       # Angle ref 
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Angle Ref vs Angle ', fontsize = self.big_font_size)
        time = np.asarray(log.attstab_time)
        angle_ref = np.asarray(log.attstab_angle_ref)
        angle = np.asarray(log.rb_euler)
        
        ax_lst[0].plot( time, angle_ref[:,0])
        ax_lst[0].plot( time, angle[:,0] )
        ax_lst[0].set_title("Roll")
        ax_lst[0].legend(("Roll Ref","Roll"))
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel('Angles [$^\circ$]')
        
        ax_lst[1].plot( time, angle_ref[:,1] )
        ax_lst[1].plot( time, angle[:,1])
        ax_lst[1].set_title("Pitch")
        ax_lst[1].legend(("Pitch Ref","Pitch"))
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel('Angles [$^\circ$]')
        
        ax_lst[2].plot( time, angle_ref[:,2])
        ax_lst[2].plot( time, angle[:,2])
        ax_lst[2].set_title("Yaw")
        ax_lst[2].legend(("Yaw Ref","Yaw"))
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel('Angles [$^\circ$]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__attstab_angle_ref.png"
        fig.savefig(fullname)
        plt.close(fig)
        
        # omega ref 
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Omega Ref vs Omega', fontsize = self.big_font_size)
        time = np.asarray(log.attstab_time)
        omega_ref = np.asarray(log.attstab_omega_ref)
        omega = np.asarray(log.rb_omegab)
        
        ax_lst[0].plot( time, omega_ref[:,0])
        ax_lst[0].plot( time, omega[:,0])
        ax_lst[0].set_title("Omega X")
        ax_lst[0].legend(("Omega X Ref","Omega X"))
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel(' Ang. Vel [$^\circ$/s]')
        
        ax_lst[1].plot( time, omega_ref[:,1] )
        ax_lst[1].plot( time, omega[:,1] )
        ax_lst[1].set_title("Omega Y")
        ax_lst[1].legend(("Omega Y Ref","Omega Y"))
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel(' Ang. Vel [$^\circ$/s]')
        
        ax_lst[2].plot( time, omega_ref[:,2])
        ax_lst[2].plot( time, omega[:,2] )
        ax_lst[2].set_title("Omega Z")
        ax_lst[2].legend(("Omega Z Ref","Omega Z"))
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel(' Ang. Vel [$^\circ$/s]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__attstab_omega_ref.png"
        fig.savefig(fullname)
        plt.close(fig)
        
        # tau ref 
        
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Tau Ref vs Tau (body frame)', fontsize = self.big_font_size)
        time = np.asarray(log.attstab_time)
        tau_ref = np.asarray(log.attstab_tau_ref)
        tau = np.asarray(log.ftau_taub)
        
        ax_lst[0].plot( time, tau_ref[:,0])
        ax_lst[0].plot( time, tau[:,0])
        ax_lst[0].set_title("Tau X")
        ax_lst[0].legend(("Tau X Ref","Tau X"))
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel(' Torque [N$\cdot$m]')
        
        ax_lst[1].plot( time, tau_ref[:,1])
        ax_lst[1].plot( time, tau[:,1])
        ax_lst[1].set_title("Tau Y")
        ax_lst[1].legend(("Tau Y Ref","Tau Y"))
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel(' Torque [N$\cdot$m]')
        
        ax_lst[2].plot( time, tau_ref[:,2])
        ax_lst[2].plot( time, tau[:,2])
        ax_lst[2].set_title("Tau Z")
        ax_lst[2].legend(("Tau Z Ref","Tau Z"))
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel(' Torque [N$\cdot$m]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__attstab_tau_ref.png"
        fig.savefig(fullname)
        plt.close(fig)
    
    def plot_posctrl(self, log):
        
         # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)
            
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Pos Ref vs Pos', fontsize = self.big_font_size)
        time = np.asarray(log.posctrl_time)
        pos_ref = np.asarray(log.posctrl_pos_ref)
        pos = np.asarray(log.rb_pos)
        
        ax_lst[0].plot( time, pos_ref[:,0])
        ax_lst[0].plot( time, pos[:,0])
        ax_lst[0].set_title(" X")
        ax_lst[0].legend((" X Ref"," X"))
        ax_lst[0].set_xlabel('Time [s]');ax_lst[0].set_ylabel(' Distance [m]')
        
        ax_lst[1].plot( time, pos_ref[:,1])
        ax_lst[1].plot( time, pos[:,1])
        ax_lst[1].set_title("Y")
        ax_lst[1].legend(("Y Ref","Y"))
        ax_lst[1].set_xlabel('Time [s]');ax_lst[1].set_ylabel(' Distance [m]')
        
        ax_lst[2].plot( time, pos_ref[:,2])
        ax_lst[2].plot( time, pos[:,2])
        ax_lst[2].set_title("Z")
        ax_lst[2].legend(("Z Ref","Z"))
        ax_lst[2].set_xlabel('Time [s]');ax_lst[2].set_ylabel(' Distance [m]')
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__posctrl_pos_ref.png"
        fig.savefig(fullname)
        plt.close(fig)