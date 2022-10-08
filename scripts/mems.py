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

""" Module implements the a mems sensor with white noise and slowly moving bias

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import random 
import math 

class mems:
    """ Holds the states and parameters of the mems sensor 
    ( either a gyroscope or an accelerometer )
    """
    
    def __init__(self,rw,rrw,bias):
    
        self.rw = rw
        """ Random Walk Parameter (ARW or VRW) """
        
        self.rrw = rrw
        """ Rate Random Walk Parameter   """
        
        self.bias = bias
        """ MEMS bias """
        
    def run_mems(self,dt,value):
        
        wn = random.gauss(0, self.rw/math.sqrt(dt))
        self.bias = self.bias + random.gauss(0, self.rrw/math.sqrt(dt))
        return (value + self.bias + wn)
            