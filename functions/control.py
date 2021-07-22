#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Control methods.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Date:       19 July 2021
Revisions: 
"""

import numpy as np
import math
from functions.guidance import refModel3

#  u = PIDpolePlacement(_x,e_v,e_int,x_d,v_d,a_d,m,d,k,wn_d,zeta_d,wn,zeta,r,v_max,sampleTime)
def PIDpolePlacement(e_x,e_v,e_int,x_d,v_d,a_d,m,d,k,wn_d,zeta_d,wn,zeta,r,v_max,sampleTime):

        # PID gains based on pole placement
        Kp = m * wn**2 - k
        Kd = m * 2 * zeta * wn - d
        Ki = (wn/10) * Kp
        
        # PID control law
        u = -Kp * e_x - Kd * e_v - Ki * e_int

        # Forward Euler integration
        e_int = e_int + sampleTime * e_x
        
        # 3rd-order reference model for smooth position, velocity and acceleration
        [x_d, v_d, a_d] = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)

        return u, e_int, x_d, v_d, a_d    
