#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Guidance methods

Author:     Thor I. Fossen
Date:       19 July 2021
Revisions: 
"""

# R = refModel3) 
def refModel3(x_d, v_d, r_f, r, wn, zeta, sampleTime):

    import numpy as np
    import math
    
    a_d = wn**2 *( r_f - x_d ) - 2 * zeta * wn * v_d
    
   # Forrward Euler integration
    x_d = x_d + sampleTime * v_d
    v_d = v_d + sampleTime * a_d
    r_f = r_f + sampleTime * wn * (r - r_f)
    
    return x_d, v_d, a_d
