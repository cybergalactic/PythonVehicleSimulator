#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kinematic functions and numerical integration.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Date:       19 July 2021
"""

# R = Rzyx(phi,theta,psi) computes the Euler angle rotation matrix R in SO(3)
# using the zyx convention
def Rzyx(phi,theta,psi):

    import numpy as np
    import math
    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)
    cpsi = math.cos(psi)
    spsi = math.sin(psi)
    
    R = np.matrix([
        [ cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth ],
        [ spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi ],
        [ -sth,      cth*sphi,                 cth*cphi ] ])

    return R

# T = Tzyx(phi,theta) computes the Euler angle attitude
# transformation matrix T using the zyx convention
def Tzyx(phi,theta):
    
    import numpy as np
    import math
    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)    

    try: 
        T = np.matrix([
            [ 1,  sphi*sth/cth,  cphi*sth/cth ],
            [ 0,  cphi,          -sphi],
            [ 0,  sphi/cth,      cphi/cth] ])
        
    except ZeroDivisionError:  
        print ("Tzyx is singular for theta = +-90 degrees." )
        
    return T

# eta = attitudeEuler(eta,nu,sampleTime) computes the generalized 
# position/Euler angles eta[k+1]
def attitudeEuler(eta,nu,sampleTime):
    
   import numpy as np
   
   p_dot   = Rzyx(eta[3], eta[4], eta[5]) * nu[0:3]
   v_dot   = Tzyx(eta[3], eta[4]) * nu[3:7]  
   eta_dot = np.vstack([p_dot, v_dot])

   # Forrward Euler integration
   eta = eta + sampleTime * eta_dot
    
   return eta
    
    