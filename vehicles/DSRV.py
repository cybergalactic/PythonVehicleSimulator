#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DSRV.py:  
 
DSRV(eta,nu,u,sampleTime) returns returns nu[k+1] of the state vector 
nu[k]  = [ 0 0 w 0 q 0]' for a deep submergence rescue vehicle (DSRV) 
L = 5.0 m, where
 w:       heave velocity (m/s)
 q:       pitch velocity (rad/s)

Input:
  u:       delta (rad), where delta is the stern plane

Reference : A.J. Healey (1992). Marine Vehicle Dynamics Lecture Notes and 
            Problem Sets, Naval Postgraduate School (NPS), Monterey, CA.

Author:     Thor I. Fossen
Date:       18 July 2021
"""
import numpy as np

# Class Vehicle
class DSRV:
    
    def __init__(self):
        # Initialize the DSRV model
        self.nu  = np.array([ [0, 0, 0, 0, 0, 0] ]).T
        self.name = "DSRV"
        self.L = 5.0
        self.U0 = 2.11
        
    def dynamics(self,eta,nu,sampleTime):
        # DSRV dynamics
        self.nu  = nu + sampleTime * np.array([ [self.U0, 0, 0, 0, 0, 0] ]).T
        
        return self.nu
        

    