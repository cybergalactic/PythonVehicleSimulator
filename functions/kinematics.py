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

import numpy as np
import math

# S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
# The cross product satisfies: a x b = S(a)b. 
def Smtrx(a):
 
    S = np.array([ 
        [ 0, -a[2], a[1] ],
        [ a[2],   0,     -a[0] ],
        [-a[1],   a[0],   0 ]  ])

    return S

# H = HMmtrx(r) computes the 6x6 system transformation matrix
# H = [eye(3)     S'
#      zeros(3,3) eye(3) ]       Property: inv(H(r)) = H(-r)
#
# If r = r_g is the vector from CO to CG, the model matrices in CO and CG
# are related by: M_CO = H(r_g)' * M_CG * H(r_g). Generalized position and
# force satisfy: eta_CO = H(r_g)' * eta_CG and tau_CO = H(r_g)' * tau_CG 
def Hmtrx(r):

    H = np.identity(6)
    H[0:3, 3:6] = Smtrx(r).T

    return H

# R = Rzyx(phi,theta,psi) computes the Euler angle rotation matrix R in SO(3)
# using the zyx convention
def Rzyx(phi,theta,psi):

    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)
    cpsi = math.cos(psi)
    spsi = math.sin(psi)
    
    R = np.array([
        [ cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth ],
        [ spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi ],
        [ -sth,      cth*sphi,                 cth*cphi ] ])

    return R

# T = Tzyx(phi,theta) computes the Euler angle attitude
# transformation matrix T using the zyx convention
def Tzyx(phi,theta):
    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)    

    try: 
        T = np.array([
            [ 1,  sphi*sth/cth,  cphi*sth/cth ],
            [ 0,  cphi,          -sphi],
            [ 0,  sphi/cth,      cphi/cth] ])
        
    except ZeroDivisionError:  
        print ("Tzyx is singular for theta = +-90 degrees." )
        
    return T

# eta = attitudeEuler(eta,nu,sampleTime) computes the generalized 
# position/Euler angles eta[k+1]
def attitudeEuler(eta,nu,sampleTime):
   
   p_dot   = np.matmul( Rzyx(eta[3], eta[4], eta[5]), nu[0:3] )
   v_dot   = np.matmul( Tzyx(eta[3], eta[4]), nu[3:7] )
   eta_dot = np.hstack([p_dot, v_dot])

   # Forward Euler integration
   eta = eta + sampleTime * eta_dot

   return eta
    
    