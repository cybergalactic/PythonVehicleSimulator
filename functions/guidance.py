#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Guidance methods.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Date:       19 July 2021
Revisions: 
"""

import numpy as np
import math

# refModel3(x_d, v_d, r_f, r, wn, zeta, sampleTime) is a 3-order reference 
# model for generation of a smooth desired position x_d, velocity v_d, and
# acceleration a_d. Inputs are natural frequency wn and relative damping zeta.
def refModel3(x_d, v_d, r_f, r, wn, zeta, sampleTime):
    
    # desired acceleration
    a_d = wn**2 *( r_f - x_d ) - 2 * zeta * wn * v_d
    
   # forward Euler integration
    x_d = x_d + sampleTime * v_d                # desired position
    v_d = v_d + sampleTime * a_d                # desired velocity
    r_f = r_f + sampleTime * wn * (r - r_f)     # LP-filtered setpoint r
    
    return x_d, v_d, a_d