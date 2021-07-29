# -*- coding: utf-8 -*-
"""
Control methods.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
"""

import numpy as np
import math
from functions.guidance import refModel3
from functions.gnc import ssa, Rzyx

# SISO PID pole placement
def PIDpolePlacement(e_int,e_x,e_v, \
    x_d,v_d,a_d,m,d,k,wn_d,zeta_d,wn,zeta,r,v_max,sampleTime):    
      
    # PID gains based on pole placement
    Kp = m * wn**2 - k
    Kd = m * 2 * zeta * wn - d
    Ki = (wn/10) * Kp

    # PID control law
    u = -Kp * e_x - Kd * e_v - Ki * e_int

    # Integral error, Euler's method
    e_int += sampleTime * e_x
    
    # 3rd-order reference model for smooth position, velocity and acceleration
    [x_d, v_d, a_d] = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)

    return u, e_int, x_d, v_d, a_d  


# MIMO nonlinear PID pole placement 
def DPpolePlacement(e_int,M3,D3,eta3,nu3, \
    x_d,y_d,psi_d,wn,zeta,eta_ref,sampleTime):  

    # PID gains based on pole placement
    Kp = wn @ wn @ M3
    Kd = 2.0 * zeta @ wn @ M3 - D3
    Ki = (1.0 / 10.0) * wn @ Kp
        
    # DP control law - setpoint regulation
    e = eta3 - eta_ref
    e[2] = ssa( e[2] )
    R = Rzyx( 0.0, 0.0, eta3[2] )
    tau = -np.matmul( (R.T @ Kp), e) \
          -np.matmul( (R.T @ Kd @ R), nu3) \
          -np.matmul( (R.T @ Ki), e_int)

    # Low-pass filters, Euler's method
    T = 5.0 * np.array([ 1/wn[0][0], 1/wn[1][1], 1/wn[2][2] ]) 
    x_d   += sampleTime  * ( eta_ref[0] - x_d )  / T[0]
    y_d   += sampleTime  * ( eta_ref[1] - y_d )  / T[1]
    psi_d += sampleTime * ( eta_ref[2] - psi_d ) / T[2]

    # Integral error, Euler's method
    e_int +=  sampleTime * e
        
    return tau, e_int, x_d, y_d, psi_d
