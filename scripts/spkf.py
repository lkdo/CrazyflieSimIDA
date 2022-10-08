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

""" Module implements the a mems sensor with white noise and slowly moving bias

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
from scipy.integrate import odeint

class SUT: 
    """ Scaled Unscented Transform """

    def __init__(self, alpha, beta, kappa, n):
        
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = n

        self.l = (self.alpha**2)*(self.n+self.kappa) - self.n # lambda parameter 

        # Init weights (constants) 
        self.W0m = self.l/(self.n + self.l)
        self.W0c = self.W0m + 1 - self.alpha**2 + self.beta
        self.Wi = 0.5/(self.n + self.l)
        """ sigma points weights """
    
    def create_points(self, x, P):
        sr_P = np.linalg.cholesky((self.n + self.l)*P)
        S = [x]
        for i in range(self.n):
            S.append(x+sr_P[:,i])
            S.append(x-sr_P[:,i])
        return S
##########################################################

class SPKF:
    """ Sigma Point Kalman Filter """

    def __init__(self,f,h,G,Q,R,x0,P0,spt,variant=0):

        self.f = f
        """ Continous state dynamics; dot(x) =  f(x,u) """

        self.h = h
        """ Measurement function y = h(x) """

        self.G = G
        """ Noise input matrix """

        self.x = x0
        """ Initial State """

        self.P = P0
        """ Initial Covariance """

        self.Q = Q
        """ Propagation noise matrix """

        self.R = R
        """ Measurement noise matrix """

        self.spt = spt
        """ Sigma point transform and parameters """

        self.n = x0.size
        """ State dimensionality """

        self.variant = variant # 0 - normal UKF, 1 - IUKF, 2 - UKFz

    def predict(self,u,dt,simple = 1):

        S = self.spt.create_points(self.x, self.P)
  
        # propagate the points 
        Sp = [ ]
        for i in range(len(S)):
            if (simple):
                # Euler faster
                Sp.append(S[i]+self.f(S[i],0,u)*dt)
            else:
                # ODE Int integration to make the state tranzition
                Y = odeint(self.f,S[i],np.array([0, dt]),args=(u,))
                Sp.append(Y[1]) # Y[0]=x(t=t0)
     
        # calculate the mean, covariance and cross-covariance of the set
        Xm = self.spt.W0m*Sp[0]
        for i in range(2*self.n):
            Xm = Xm + self.spt.Wi*Sp[i+1]

        Cx = self.spt.W0c*np.outer(Sp[0]-Xm,Sp[0]-Xm)+dt*self.Q
        for i in range(2*self.n):
            Cx = Cx + self.spt.Wi*np.outer(Sp[i+1]-Xm,Sp[i+1]-Xm)

        self.x = Xm
        self.P = Cx

    def update(self,meas,var=0):

        X = self.spt.create_points(self.x, self.P)
        
        Y = [ ]
        for i in range(2*self.n+1):
            Y.append(self.h(X[i]))

        # calculate the mean and covariance of the set
        if (self.variant == 2): # UKFz
            Ym = self.h(self.x)
        else:  
            Ym = self.spt.W0m*Y[0]
            for i in range(2*self.n):
                Ym = Ym + self.spt.Wi*Y[i+1]
        
        Cy = self.spt.W0c*np.outer(Y[0]-Ym,Y[0]-Ym)
        Cxy = self.spt.W0c*np.outer(X[0]-self.x,Y[0]-Ym)
        for i in range(2*self.n):
            Cy = Cy + self.spt.Wi*np.outer(Y[i+1]-Ym,Y[i+1]-Ym)
            Cxy = Cxy + self.spt.Wi*np.outer(X[i+1]-self.x,Y[i+1]-Ym)

        # Kalman Gain
        K = Cxy@np.linalg.inv(Cy + self.R)

        # Update mean and covarince
        if (self.variant == 1): # IUKF
            inn = meas - self.h(self.x)
        else:
            inn = meas - Ym

        self.x = self.x + K@inn
        
        if (var == 0):
            # Simple Covariance Update
            self.P = self.P - K@(Cy+self.R)@np.transpose(K)
        else:
            # -> Joseph Form Covariance Update
            # H = stochastic linearization derived from the fact that Cxy = PH' in the linear case [Skoglund, Gustafsson, Hendeby - 2019]
            self.H = Cxy.transpose()@np.linalg.inv(self.P)
            IKH =  np.eye(self.n) - K@self.H
            self.P = IKH@self.P@IKH.transpose()+K@self.R@K.transpose() # Joseph Form
##########################################################

