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

""" Panda3D related stuff: Keyboard controls class and game engine class """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

# Global imports 
import math 

# Panda 3D imports 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.showbase import DirectObject
from panda3d.core import LQuaternionf
from direct.gui.OnscreenText import OnscreenText

#local imports 
import envir
import utils

######################################################
class ReadKeys(DirectObject.DirectObject):
    """ Panda3D class to catch keyboard inputs for commanding the quadrotor model  """
   
    def __init__(self,ref,ctrl_mode,panda3D_app):
        
        self.ref = ref # [ thrust, omega_ref[0], omega_ref[1], omega_ref[2]  ]
        self.ctrl_mode = ctrl_mode
        self.panda3D_app = panda3D_app
        self.exitpressed = False
                
        self.accept("time-arrow_up", self.call_fw)
        self.accept("time-arrow_up-repeat", self.call_fw)
        
        self.accept("time-arrow_down", self.call_bw)
        self.accept("time-arrow_down-repeat", self.call_bw)
        
        self.accept("time-arrow_left", self.call_left)
        self.accept("time-arrow_left-repeat", self.call_left)
        
        self.accept("time-arrow_right", self.call_right)
        self.accept("time-arrow_right-repeat", self.call_right)
        
        self.accept("time-a", self.call_dz)
        self.accept("time-a-repeat", self.call_dz)
        
        self.accept("time-z", self.call_dz_neg)
        self.accept("time-z-repeat", self.call_dz_neg)
        
        self.accept("time-j",self.call_yaw_cw)
        self.accept("time-j-repeat",self.call_yaw_cw)
        
        self.accept("time-g",self.call_yaw_ccw)
        self.accept("time-g-repeat",self.call_yaw_ccw)
        
        self.accept("time-y",self.call_camera_yaw)
        
        self.accept ("time-escape",self.call_exitpressed)
        
    def call_fw(self, when):
        if self.ctrl_mode == 1: 
           self.ref[2] = self.ref[2] + 5*math.pi/180.0
        elif self.ctrl_mode == 2:
           self.ref[2] = self.ref[2] + 1*math.pi/180.0 # ref are in radians
           if ( self.ref[2] > math.pi/180.0 * 80 ):
                self.ref[2] = math.pi/180.0 * 80   
        elif self.ctrl_mode == 3:
           if self.panda3D_app.camera_yaw is False:
               self.ref[0] += 1 # distance x-axis meters
           else:
               h=self.panda3D_app.qrb.rpy[2]
               self.ref[0] += 1*math.cos(h) # distance x-axis meters
               self.ref[1] += 1*math.sin(h)
               
        self.panda3D_app.screenText_ref(self.ref)
         
    def call_bw(self, when):
        if self.ctrl_mode == 1: 
           self.ref[2] = self.ref[2] - 5*math.pi/180.0
        elif self.ctrl_mode == 2:
           self.ref[2] = self.ref[2] - math.pi/180.0 * 1  # ref are in radians
           if ( self.ref[2] < - math.pi/180.0 * 80 ):
                self.ref[2] = - math.pi/180.0 * 80   
        elif self.ctrl_mode == 3:
            if self.panda3D_app.camera_yaw is False:
                self.ref[0] += -1 # distance x-axis meters            
            else:
                h=self.panda3D_app.qrb.rpy[2]
                self.ref[0] += -1*math.cos(h) # distance x-axis meters
                self.ref[1] += -1*math.sin(h)
               
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_left(self, when):
        if self.ctrl_mode == 1:
            self.ref[1] = self.ref[1] - 5*math.pi/180.0
        elif self.ctrl_mode == 2:    
            self.ref[1] = self.ref[1] -  math.pi/180.0 * 1  # ref are in radians
            if ( self.ref[1] < - math.pi/180.0 * 80 ):
                self.ref[1] = - math.pi/180.0 * 80
        elif self.ctrl_mode == 3:
            if self.panda3D_app.camera_yaw is False:
                self.ref[1] += 1
            else:
                h=self.panda3D_app.qrb.rpy[2]
                self.ref[0] += -1*math.sin(h) # distance x-axis meters
                self.ref[1] += 1*math.cos(h)
               
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_right(self, when):
        # decrease (wing right down) roll rate with "x" rad/sec
        if self.ctrl_mode == 1:
            self.ref[1] = self.ref[1] + 5*math.pi/180.0
        elif self.ctrl_mode == 2:
            self.ref[1] = self.ref[1] +  math.pi/180.0 * 1  # ref are in radians
            if ( self.ref[1] >  math.pi/180.0 * 80 ):
                self.ref[1] = math.pi/180.0 * 80
        elif self.ctrl_mode == 3:
            if self.panda3D_app.camera_yaw is False:
                self.ref[1] += -1
            else:
                h=self.panda3D_app.qrb.rpy[2]
                self.ref[0] += 1*math.sin(h) # distance x-axis meters
                self.ref[1] += -1*math.cos(h)
                
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_dz(self, when):
        if self.ctrl_mode == 1 or self.ctrl_mode == 2:
            self.ref[0] += 0.0001*envir.g # 0.1 gram more thrust
            if self.ref[0] > 0.8*0.638:
               self.ref[0] = 0.8*0.638
        elif self.ctrl_mode == 3:
            self.ref[2] += 1                 # meters 
       
        self.panda3D_app.screenText_ref(self.ref)

    def call_dz_neg(self, when):
        if self.ctrl_mode == 1 or self.ctrl_mode == 2:
            self.ref[0] -=  0.0001*envir.g # 0.1 gram less thrust
            if self.ref[0] < 0:
                self.ref[0] = 0 
        elif self.ctrl_mode == 3:
            self.ref[2] += -1           # meters
        
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_yaw_cw(self, when):
        if self.ctrl_mode == 1:
            self.ref[3] = self.ref[3] - 5*math.pi/180.0
        elif self.ctrl_mode == 2 or self.ctrl_mode == 3:
            self.ref[3] = self.ref[3] - 1*math.pi/180.0  # ref are in radians
            if  (self.ref[3] < -math.pi):
                  self.ref[3] = self.ref[3] + 2*math.pi
        
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_yaw_ccw(self, when):
        if self.ctrl_mode == 1:
            self.ref[3] = self.ref[3] + 5*math.pi/180.0
        elif self.ctrl_mode == 2 or self.ctrl_mode == 3:
            self.ref[3] = self.ref[3] + 1*math.pi/180.0  # ref are in radians
            if  (self.ref[3] > math.pi):
                  self.ref[3] = self.ref[3] - 2*math.pi
        
        self.panda3D_app.screenText_ref(self.ref)
        
    def call_exitpressed(self,when):
        self.exitpressed = True
        
    def call_camera_yaw(self,when):
        self.panda3D_app.camera_yaw = not self.panda3D_app.camera_yaw
        
########################################################        
class Panda3DApp(ShowBase):
    """ Main class of the game engine Panda3D """
   
    def __init__(self, plus, qrb, ref, ctrl_mode = 1, camera_yaw = False ):
        
        self.qrb = qrb 
        """ The rigid Body object """
        
        self.ctrl_mode = ctrl_mode
        
        ShowBase.__init__(self)
		
        # Load the environment model.
        self.scene = self.loader.loadModel("../models/environment") 
           
	    # Reparent the model to render.
        self.scene.reparentTo(self.render)
    
	    # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
		
        if plus is False:
            self.quadrotor = self.loader.loadModel("./res/CF21_cross")
        else:
            self.quadrotor = self.loader.loadModel("./res/CF21_plus")
            
        self.quadrotor.setScale(1, 1, 1)
        self.quadrotor.reparentTo(self.render)
        self.quadrotor.setPos(0,0,3)
            
        # Add the move  procedure to the task manager.
        self.taskMgr.add(self.moveActorTask, "MoveQuadTask")
        
		# Add the followQuadCameraTask  procedure to the task manager.
        self.taskMgr.add(self.followQuadCameraTask, "followQuadCameraTask")
        
        monospaced_font = self.loader.loadFont("cmtt12.egg")
        self.textObject_pos = OnscreenText("Position and Attitude (real)", pos = (0, +0.8), scale = 0.07,
            fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True, font = monospaced_font)
        self.textObject_pos_filter = OnscreenText("Position and Attitude (kal) ", pos = (0, +0.7), scale = 0.07,
            fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True, font = monospaced_font)
        if self.ctrl_mode == 1 or self.ctrl_mode ==2 :
            text = "T={0: 5.3f},R={1: 3.1f},P={2: 3.1f},Y={3: 3.1f}".format(ref[0],
                ref[1]*180/math.pi,ref[2]*180/math.pi,ref[3]*180/math.pi)
        elif self.ctrl_mode == 3 :
            text = "X={0: 3.1f},Y={1: 3.1f},Z={2: 3.1f},Y={3: 4.2f}".format(ref[0],
                ref[1],ref[2],ref[3]*180/math.pi)
            
        self.textObject_ref = OnscreenText(text, pos = (0, -0.8), scale = 0.07,
            fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True, font = monospaced_font)

        self.camera_yaw = camera_yaw
        
    # Define a procedure to move the camera.
    
    def followQuadCameraTask(self, task):
        h=self.qrb.rpy[2]
        if self.camera_yaw is False :
            self.camera.setPos(self.quadrotor.getX()-20, self.quadrotor.getY(), self.quadrotor.getZ()+3)
        else:
            self.camera.setPos(self.quadrotor.getX()-20*math.cos(h), 
                                     self.quadrotor.getY()-20*math.sin(h), 
                                     self.quadrotor.getZ()+3)
        self.camera.lookAt(self.quadrotor)
        return Task.cont

    # Define a procedure to move the panda actor 
    
    def moveActorTask(self, task):
        pos = self.qrb.pos
        q = LQuaternionf(self.qrb.q[0],self.qrb.q[1],self.qrb.q[2],self.qrb.q[3])
        self.quadrotor.setQuat(q)
        self.quadrotor.setPos(pos[0],pos[1],pos[2])
        return Task.cont

    # function to write on screen 
    
    def screenText_ref(self,ref):
        if self.ctrl_mode == 1 or self.ctrl_mode ==2 :
            text = "REF T={0: 5.3f},R={1: 3.1f},P={2: 3.1f},Y={3: 3.1f}".format(ref[0],
                ref[1]*180/math.pi,ref[2]*180/math.pi,ref[3]*180/math.pi)
        elif self.ctrl_mode == 3:
            text = "REF X={0: 3.1f},Y={1: 3.1f},Z={2: 3.1f},Yaw={3: 3.1f}".format(ref[0],
                ref[1],ref[2],ref[3]*180/math.pi)
        self.textObject_ref.text = text


    def screenText_pos(self,pos,rpy,pos_f,rpy_f):
         text = "REAL X={0:5.2f},Y={1:5.2f},Z={2:5.2f},RPY={3:5.2f},{4:5.2f},{5:5.2f}".format(
            pos[0],pos[1],pos[2],rpy[0]*180/math.pi,rpy[1]*180/math.pi,rpy[2]*180/math.pi)
         self.textObject_pos.text = text
         text = "KF   X={0:5.2f},Y={1:5.2f},Z={2:5.2f},RPY={3:5.2f},{4:5.2f},{5:5.2f}".format(
            pos_f[0],pos_f[1],pos_f[2],rpy_f[0]*180/math.pi,rpy_f[1]*180/math.pi,rpy_f[2]*180/math.pi)
         self.textObject_pos_filter.text = text
