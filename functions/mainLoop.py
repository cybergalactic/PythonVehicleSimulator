#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 16:55:20 2021

@author: thor
"""

# Function simulate
def simulate(N,sampleTime,x):
    
    import numpy as np
    
    t = 0
    simData = np.empty((0,2), float)
    
    for i in range(0,N+1):
        t = i * sampleTime
        
        simData = np.append(simData,[[x,2*x]],axis=0)
        
        x = x + sampleTime * (-x + 1)

    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData)