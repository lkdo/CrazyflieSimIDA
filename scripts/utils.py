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

""" Useful functions 

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import math
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Rot

def cay(A):
    n = A.shape[0]
    return inv(np.eye(n)+A)@(np.eye(n)-A)

def Rx(theta):
  return np.array([[ 1.0, 0.0           , 0.0           ],
                   [ 0.0, math.cos(theta), -math.sin(theta)],
                   [ 0.0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
  return np.array([[ math.cos(theta), 0.0, math.sin(theta)],
                   [ 0.0            , 1.0, 0.0           ],
                   [ -math.sin(theta), 0.0, math.cos(theta)]])
  
def Rz(theta):
  return np.array([[ math.cos(theta), -math.sin(theta), 0.0],
                   [ math.sin(theta), math.cos(theta) , 0.0 ],
                   [ 0.0          , 0.0            , 1.0 ]])

def rpy2rotm2(rpy):
    r = rpy[0]
    p = rpy[1]
    y = rpy[2]
    R = Rz(y)@Ry(p)@Rx(r)
    #normalize the rotation matrix
    R = cay((np.transpose(R) - R)/(1.0+np.trace(R)))
    return R

def rpy2rotm(rpy):
    r = Rot.from_euler('xyz',rpy)
    return r.as_matrix()

def rotm2rpy(R,sol=1):
#   That is input matrix R = Rx(psi)*Ry(theta)*Rz(phi)
    r = Rot.from_matrix(R)
    return r.as_euler('xyz')

def rpy2q(rpy):
    QR = np.array([math.cos(rpy[0]/2.0),math.sin(rpy[0]/2.0),0,0])
    QP = np.array([math.cos(rpy[1]/2.0),0,math.sin(rpy[1]/2.0),0])
    QY = np.array([math.cos(rpy[2]/2.0),0,0,math.sin(rpy[2]/2.0)])
    return quaternion_multiply(QR,quaternion_multiply(QP,QY))
##########################################################

def rotm2rpy2(R, sol=1):  #  R.as_euler('xyz')
    """ Transforms a rotation matrix to Euler angles
    That is input matrix R = Rz(phi)*Ry(theta)*Rx(psi), 
    and the output is [psi, theta, phi]. 
    Implements the pseudocode from
    "Computing Euler angles from a rotation matrix", 
    by Gregory G. Slabaugh
    """
    if ((R[3-1,1-1]!=1) and (R[3-1,1-1]!=-1)):
        theta1 = -math.asin(R[3-1,1-1])
        theta2 = math.pi - theta1
        psi1 = math.atan2(R[3-1,2-1]/math.cos(theta1),
                          R[3-1,3-1]/math.cos(theta1))
        psi2 = math.atan2(R[3-1,2-1]/math.cos(theta2),
                          R[3-1,3-1]/math.cos(theta2))
        phi1 = math.atan2(R[2-1,1-1]/math.cos(theta1),
                          R[1-1,1-1]/math.cos(theta1))
        phi2 = math.atan2(R[2-1,1-1]/math.cos(theta2),
                          R[1-1,1-1]/math.cos(theta2))
        # Choose one set of rotations
        if sol == 1:
            return np.array([psi1,theta1,phi1])
        else:
            return np.array([psi2,theta2,phi2])
    else:
        phi = 0 # can be anything 
        if R[3-1,1-1] == -1:
            theta = math.pi/2
            psi = phi + math.atan2(R[1-1,2-1],R[1-1,3-1])
        else:
            theta = -math.pi/2
            psi = -phi + math.atan2(-R[1-1,2-1],R[1-1,3-1])
        return np.array([psi,theta,phi])
##########################################################

def quat2rotm(q): 
    """ Takes a quaternion and returns the rotation matrix"""
    
    return 2*np.array([
                      [ q[0]**2+q[1]**2-0.5,
                       (q[1]*q[2]-q[0]*q[3]), 
                       (q[1]*q[3]+q[0]*q[2])   ],
                      [ (q[1]*q[2]+q[0]*q[3]),
                        (q[0]**2+q[2]**2-0.5),
                        (-q[0]*q[1]+q[2]*q[3]) ],
                      [ (q[1]*q[3]-q[0]*q[2]), 
                        (q[2]*q[3]+q[0]*q[1]),
                        q[0]**2+q[3]**2-0.5 ] 
                    ])  
##########################################################

    
def skew(X):
    """  Returns the skew-symmetric matrix form of the input vector """
    return np.array([[0,-X[3-1],X[2-1]],[X[3-1],0,-X[1-1]],[-X[2-1],X[1-1],0]])
##########################################################
    

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                      x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                      x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
##########################################################


def build_signal_step(t_start, t_end, base_value, *arg):

     small = 0.00001
     n = len(arg)
     
     ref = np.zeros([2+4*n,2])
     
     k = 0
     ref[k,0] = t_start
     ref[k,1] = base_value
     k += 1
     
     for i in range(n):
         
         ref[k,0] = arg[i].t_start - small
         ref[k,1] = base_value
         k += 1
     
         ref[k,0] = arg[i].t_start
         ref[k,1] = base_value + arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end 
         ref[k,1] = base_value + arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end + small 
         ref[k,1] = base_value
         k += 1 

     ref[k,0] = t_end
     ref[k,1] = base_value
     k += 1

     return ref 
 
def build_signal_ramp(t_start, t_end, base_value, *arg):

     n = len(arg)
     
     ref = np.zeros([2+3*n,2])
     
     k = 0
     ref[k,0] = t_start
     ref[k,1] = base_value
     k += 1
     
     for i in range(n):
         
         ref[k,0] = arg[i].t_start 
         ref[k,1] = base_value
         k += 1
     
         ref[k,0] = arg[i].t_start + (arg[i].t_end - arg[i].t_start)/2
         ref[k,1] = base_value+arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end 
         ref[k,1] = base_value
         k += 1 

     ref[k,0] = t_end
     ref[k,1] = base_value
     k += 1

     return ref     
    
    
def build_signal_sin(t_start, t_end, base_value, *arg):

    n = len(arg)
    
    N = 2
    for i in range(n):
        N += arg[i].no_periods*arg[i].no_points_per_period
    
    ref = np.zeros([N,2])
    
    k = 0
    ref[k,0] = t_start
    ref[k,1] = base_value
    k += 1
     
    for i in range(n):
         x = np.linspace(0,arg[i].no_periods*arg[i].period,arg[i].no_periods*arg[i].no_points_per_period)
         sinx = base_value + arg[i].amplitude*np.sin(2*math.pi/arg[i].period*x)
         ref[k:k+len(x),0]  = arg[i].t_start + x
         ref[k:k+len(x),1]  = sinx 
         k = k +len(x)
         
    ref[k,0] = t_end
    ref[k,1] = base_value
    k += 1
    
    return ref
    
def give_signal(ref, t):
    
    k = 0
    n = len(ref)
    while (ref[k,0] < t) and (k < n-1):
        k+=1 
    
    if (k==0) or (k == n):
        return ref[k,1] 
    else:
        return( ((ref[k,1]-ref[k-1,1])*t + (ref[k-1,1]*ref[k,0]-ref[k,1]*ref[k-1,0]))
                     /(ref[k,0] - ref[k-1,0]) )
        
def clampRotation(angle):
    
    while (angle > math.pi):
      angle -= 2*math.pi
    
    while (angle < -math.pi):
      angle += 2*math.pi
    
    return angle

def wrap_euler(euler):
    for i in range(3):    
        euler[i] = clampRotation(euler[i])
    return euler     
