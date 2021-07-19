#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DSRV.py:  
 
DSRV(eta,nu,u,sampleTime) returns returns nu[k+1] of the state vector 
nu[k]  = [ 0 0 w 0 q 0]' for a deep submergence rescue vehicle (DSRV) 
L = 5.0 m, where
 w:        heave velocity (m/s)
 q:        pitch velocity (rad/s)

Input:
  u:       delta_s (rad), where delta_s is the stern plane

References: 
  A.J. Healey (1992). Marine Vehicle Dynamics Lecture Notes and 
    Problem Sets, Naval Postgraduate School (NPS), Monterey, CA.
  T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
     Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
Date:       18 July 2021
"""
import numpy as np
import math

# Class Vehicle
class DSRV:
    """
    DSRV()                      default control system, step input
    DSRV('deptAutopilot',z_d)   depth autopilot, desired depth (m)
    DSRV('stepInput',delta_c)   step input, stern plane (deg)
    """        
    def __init__(self, controlSystem = 'depthAutopilot', r = 10):
                            
        if (controlSystem == 'depthAutopilot'):
            self.controlDescription = 'Depth autopilot, setpoint z_d = ' + str(r) + ' (m)'
            
        elif (controlSystem == 'stepInput'):
            self.controlDescription = 'Step input, delta_s = ' + str(r) + ' (deg)'
 
        else:  
            self.controlDescription = "ERROR, legal options {depthAutopilot, stepInput}" 
            controlSystem = 'stepInput'  
      
        self.ref = r
        self.controlMode = controlSystem
                    
        # Initialize the DSRV model
        self.name = "DSRV"
        self.L = 5.0        # Length
        
        self.deltaMax = 30  # max stern plane angle (deg)     
        self.controls = ['Stern plane (deg)']
        self.dimU = len(self.controls)
        
        self.U0 = 4.11      # Cruise speed: 4.11 m/s = 8 knots 
        self.W0 = 0
        self.nu  = np.array([ [self.U0, 0, self.W0, 0, 0, 0] ]).T
        
        # Non-dimensional mass matrix 
        Iy  =  0.001925
        m   =  0.036391
        Mqdot  = -0.001573;   Zqdot  = -0.000130
        Mwdot  = -0.000146;   Zwdot  = -0.031545
        
        self.m11 = m - Zwdot
        self.m12 = -Zqdot
        self.m22 = Iy - Mqdot
        self.m21 = -Mwdot
        
        self.detM = self.m11 * self.m22 - self.m12 * self.m21;
        
        #  Non-dimensional hydrodynamic derivatives
        self.Mq     = -0.01131;    self.Zq     = -0.017455
        self.Mw     =  0.011175;   self.Zw     = -0.043938
        self.Mdelta = -0.012797
        self.Zdelta = 0.027695
        
        # Depth autopilot
        self.z_int = 0           # integral state
    
        wn = 1                   # PID pole placement
        zeta = 1
        
        self.Kp = self.m11 * wn**2
        self.Kd = self.m11 * 2 * wn * zeta
        self.Ki = (wn/10) * self.Kp
        
        # Reference model
        self.wn_d = wn / 5
        self.z_d = 0
        self.w_d = 0
        
    def __del__(self):
        pass
        
    def dynamics(self,eta,nu,u,sampleTime):
        
        # states and inputs: eta[k], nu[k], u[k]
        delta = u[0]
        w     = nu[2]
        q     = nu[4] 
        theta = eta[4]
        
        # Speed
        U = math.sqrt( self.U0**2 + (self.W0 + w)**2 )
        
        # Speed dependet pitch moment
        Mtheta = -0.156276 / U**2
        
        # Rudder saturation
        if ( abs(delta) >= self.deltaMax * math.pi/180 ):
            delta = np.sign(delta) * self.deltaMax * math.pi/180;

        # Forces and moments        
        Z = self.Zq * q + self.Zw * w + self.Zdelta * delta
        M = self.Mq * q + self.Mw * w + Mtheta * theta + self.Mdelta * delta
            
        # State derivatives (with dimension)
        nu_dot = np.zeros((6, 1))
        nu_dot[2] = (  self.m22 * Z - self.m12 * M) / self.detM
        nu_dot[4] = ( -self.m21 * Z + self.m11 * M) / self.detM             
        
        # Forward Euler integration
        self.nu  = nu + sampleTime * nu_dot
        
        # Cruise speed (constant)
        self.nu[0] = self.U0;
        self.nu[2] = self.W0;
        
        return self.nu
    
    
    def stepInput(self,t):
       
        self.u = np.zeros([self.dimU,1])
        delta_c = (math.pi/180) * self.ref
        
        self.u[0] = delta_c       
        if t > 30:
            self.u[0] = -delta_c 
        if t > 50:
            self.u[0] = 0
            
        return self.u                

    def depthAutopilot(self,eta,nu,sampleTime):
        
        self.u = np.zeros([self.dimU,1])
        
        z = eta[2]  
        w = nu[2]
        z_err = z - self.z_d 
        w_err = w - self.w_d 
        
        self.u[0] = -self.Kp * z_err - self.Kd * w_err - self.Ki * self.z_int
                    
        self.z_d = self.z_d + sampleTime * self.w_d
        self.w_d = self.w_d + sampleTime * \
           ( self.wn_d**2 *( self.ref - self.z_d ) - 2 * self.wn_d * self.w_d )
        
        self.z_int = self.z_int + sampleTime * z_err

        return self.u    
        

    