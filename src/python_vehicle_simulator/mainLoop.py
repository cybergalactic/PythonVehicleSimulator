# -*- coding: utf-8 -*-
"""
Main simulation loop.

Author:     Thor I. Fossen
"""

from .gnc import attitudeEuler
import numpy as np

# Function simInfo(vehicle)
def simInfo(vehicle):
    print('\nThe Python Vehicle Simulator:')
    print('Vehicle:         %s, L = %s' % (vehicle.name, vehicle.L))
    print('Control system:  %s' % (vehicle.controlDescription))

        
# Function simulate(N, sampleTime, vehicle)
def simulate(N, sampleTime, vehicle):
    
    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time

    # Initial state vectors
    eta = np.array( [0, 0, 0, 0, 0,0], float)    # position/attitude, user editable
    nu = vehicle.nu                              # velocity, defined by vehicle class
    u_actual = vehicle.u_actual                  # actual inputs, defined by vehicle class  

    simInfo(vehicle)            # print simulation info

    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)
    
    # Simulator for-loop
    for i in range(0,N+1):
        
        t = i * sampleTime      # simulation time
        
        # Vehicle specific control systems
        if (vehicle.controlMode == 'depthAutopilot'):
            u_control = vehicle.depthAutopilot(eta,nu,sampleTime)
        elif (vehicle.controlMode == 'headingAutopilot'):
            u_control = vehicle.headingAutopilot(eta,nu,sampleTime)   
        elif (vehicle.controlMode == 'DPcontrol'):
            u_control = vehicle.DPcontrol(eta,nu,sampleTime)                   
        elif (vehicle.controlMode == 'stepInput'):
            u_control = vehicle.stepInput(t)          
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,sampleTime)
        eta = attitudeEuler(eta,nu,sampleTime)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData)
