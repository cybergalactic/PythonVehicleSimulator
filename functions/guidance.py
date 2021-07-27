# -*- coding: utf-8 -*-
"""
Guidance algorithms.
    
[x_d,v_d,a_d] = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)
    3rd-order reference model with saturation

---
Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Date:       25 July 2021
Revisions: 
"""

import numpy as np
import math

# refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime) is a 3-order reference 
# model for generation of a smooth desired position x_d, velocity |v_d| < v_max , and
# acceleration a_d. Inputs are natural frequency wn_d and relative damping zeta_d.
def refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime):

    # velocity saturation
    if (v_d > v_max):
        v_d = v_max
    elif (v_d < -v_max): 
        v_d = -v_max
    
    # desired "jerk"
    j_d = wn_d**3 * (r - x_d) - (2*zeta_d+1) * wn_d**2 * v_d - (2*zeta_d+1) * wn_d * a_d

   # forward Euler integration
    x_d = x_d + sampleTime * v_d                # desired position
    v_d = v_d + sampleTime * a_d                # desired velocity
    a_d = a_d + sampleTime * j_d                # desired acceleration 
    
    return x_d, v_d, a_d