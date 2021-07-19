#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 16:48:53 2021

@author: thor
"""

# plotVehicleStates(simTime, simData) plots the 6-DOF vehicle 
# position/attitude and velocities versus time
def plotVehicleStates(simTime, simData):
    
    import matplotlib.pyplot as plt
    import numpy as np
    import math
    
    # configuration parameters
    legendSize = 10                     # plot legend size
    dpiSize = 600                       # figure dpi size
    figSize = (14,7)                    # figure size in inches
    
    # constants    
    R2D = 180 / math.pi                 # radians to degrees
    
    # time vector
    t = simTime
        
    # state vectors
    x = simData[:,0]
    y = simData[:,1]
    z = simData[:,2]
    phi   = R2D * simData[:,3]    
    theta = R2D * simData[:,4]  
    psi   = R2D * simData[:,5]  
    u = simData[:,6]    
    v = simData[:,7]  
    w = simData[:,8]  
    p = R2D * simData[:,9]    
    q = R2D * simData[:,10]  
    r = R2D * simData[:,11] 
   
    # speed
    U = np.sqrt( np.multiply(u,u) + np.multiply(v,v) + np.multiply(w,w) )
     
    beta_c  = R2D * np.arctan2(v,u)         # crab angle
    alpha_c = R2D * np.arctan2(w,u)         # flight path angle
    chi     = psi + beta_c                  # course angle
    
    # plots
    plt.figure(figsize=figSize, dpi=dpiSize)
     
    plt.subplot(3, 3, 1)
    plt.plot(y, x)
    plt.legend(['North-East positions'],fontsize=legendSize) 
    
    plt.subplot(3, 3, 2)
    plt.plot(t, z)
    plt.legend(['Depth'],fontsize=legendSize) 
    
    plt.title('Vehicle states', fontsize=12)

    plt.subplot(3, 3, 3)
    plt.plot(t, phi, t, theta)   
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Roll angle','Pitch angle'],fontsize=legendSize) 
    
    plt.subplot(3, 3, 4)
    plt.plot(t, U)
    plt.legend(['Speed (SOG)'],fontsize=legendSize)    

    plt.subplot(3, 3, 5)
    plt.plot(t, chi)   
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Course angle (COG)'],fontsize=legendSize)    
    
    plt.subplot(3, 3, 6)
    plt.plot(t, beta_c, t, psi)   
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Crab angle','Yaw angle'],fontsize=legendSize)
    
    plt.subplot(3, 3, 7)
    plt.plot(t, u, t, v, t, w)  
    plt.xlabel('Time (s)', fontsize=12)     
    plt.legend(['Surge velocity','Sway velocity','Heave velocity'],
               fontsize=legendSize)   

    plt.subplot(3, 3, 8)
    plt.plot(t, p, t, q, t, r)
    plt.xlabel('Time (s)', fontsize=12) 
    plt.legend(['Roll rate','Pitch rate','Yaw rate'],fontsize=legendSize)
    
    plt.subplot(3, 3, 9)
    plt.plot(t, alpha_c, t, theta)
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Flight path angle','Pitch angle'],fontsize=legendSize)      
    
    plt.show()
    