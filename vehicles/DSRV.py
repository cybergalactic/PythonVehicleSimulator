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
from functions.control import PIDpolePlacement

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
        self.wn = 1              # PID pole placement
        self.zeta = 1
        
        # Reference model
        self.z_d = 0            # position, velocity and acc. states
        self.w_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5
        self.zeta_d = 1        
        
    def __del__(self):
        pass
        
    def dynamics(self,eta,nu,u,sampleTime):
        """
        nu = dynamics(eta,nu,u,sampleTime) integrates the DSRV
        equations of motion.
        """       
        # states and inputs: eta[k], nu[k], u[k]
        delta = u[0]
        w     = nu[2]
        q     = nu[4] 
        theta = eta[4]
        
        # Speed
        U = math.sqrt( self.U0**2 + (self.W0 + w)**2 )
        
        # Speed dependent pitch moment
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
        
        return self.nu
    
    
    def stepInput(self,t):
        """
        delta_c = stepInput(t) generates stern plane step inputs.
        """          
        self.u = np.zeros([self.dimU,1])
        delta_c = (math.pi/180) * self.ref
        
        self.u[0] = delta_c       
        if t > 30:
            self.u[0] = -delta_c 
        if t > 50:
            self.u[0] = 0
            
        return self.u                

    def depthAutopilot(self,eta,nu,sampleTime):
        """
        delta_c = depthAutopilot(eta,nu,sampleTime) is a PID controller for 
        automatic depth control based on pole placement.
        """          
        self.u = np.zeros([self.dimU,1])
        
        w_max = 1                   # maximum heave velocity

        z = eta[2]                  # heave position
        w = nu[2]                   # heave velocity
        e_z = z - self.z_d          # heave position tracking error
        e_w = w - self.w_d          # heave velocity tracking error
        r = self.ref                # heave setpoint
    
        wn = self.wn                # PID natural frequency
        zeta = self.zeta            # PID natural relative damping factor
        wn_d = self.wn_d            # reference model natural frequency
        zeta_d = self.zeta_d        # reference model relative damping factor

        m = self.m11                # mass in heave including added mass
        d = 0                 
        k = 0

        # PID feedback controller with 3rd-order reference model
        [self.u, self.z_int, self.z_d, self.w_d, self.a_d] = \
            PIDpolePlacement( e_z, e_w, self.z_int,self.z_d, self.w_d, self.a_d, \
            m, d, k, wn_d, zeta_d, wn, zeta, r, w_max, sampleTime )
    
        return self.u    
        

    