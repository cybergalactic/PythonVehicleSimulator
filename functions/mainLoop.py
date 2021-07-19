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
    
    dimU = 10                   # maximum number of control inputs
    DOF = 6                     # degrees of freedom
    t = 0                       # intial simulation time

    # initialization of table used for simulation data
    simData = np.empty( [0, 2*DOF + dimU], float)
    
    
    # Intitial states
    eta = np.array([ [0, 0, 0, 0, 0, 0] ]).T    
    nu = vehicle.nu
    
    # simulator for-loop
    for i in range(0,N+1):
        
        t = i * sampleTime      # simulation time
        
        # tau = np.array([ [1.0, 0.1, 0, 0.1, 0.2, 0.3] ]).T
        u = np.zeros([dimU,1])
        u[0] = 20/57
        if t > 20:
            u[0] = 0        
        
        # store simulation data in simData
        simData = np.vstack( [ simData, np.vstack([eta, nu, u]).transpose() ])
        
        #nu = DSRV(eta,nu,u,sampleTime)
        nu = vehicle.dynamics(eta,nu,sampleTime)
        eta = attitudeEuler(eta,nu,sampleTime)

    # store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData)
