#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main simulation loop.

Author:     Thor I. Fossen
Date:       19 July 2021
"""

from .kinematics import attitudeEuler
import numpy as np
        
# Function simulate(N, sampleTime, eta, nu)
def simulate(N, sampleTime, vehicle):
    
    DOF = 6                     # degrees of freedom
    t = 0                       # intial simulation time

    # initialization of table used for simulation data
    simData = np.empty( [0, 2*DOF + vehicle.dimU], float)
    
    # Intitial states
    eta = np.array([ [0, 0, 0, 0, 0, 0] ]).T    
    nu = vehicle.nu
    
    # simulator for-loop
    for i in range(0,N+1):
        
        t = i * sampleTime      # simulation time
        
        if (vehicle.controlMode == 'depthAutopilot'):
            u = vehicle.depthAutopilot(eta,nu,sampleTime)
        elif (vehicle.controlMode == 'stepInput'):
            u = vehicle.stepInput(t)          
        
        # store simulation data in simData
        simData = np.vstack( [ simData, np.vstack([eta, nu, u]).transpose() ])
        
        # Propagate vehicle and attitude dynamics
        nu  = vehicle.dynamics(eta,nu,u,sampleTime)
        eta = attitudeEuler(eta,nu,sampleTime)

    # store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData)