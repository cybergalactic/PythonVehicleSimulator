#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulator plotting functions.

Author:     Thor I. Fossen
Date:       19 July 2021
"""

import math
import matplotlib.pyplot as plt
import numpy as np

legendSize = 10                     # legend size
figSize1 = [30, 20]                 # figure1 size in cm
figSize2 = [25, 15]                 # figure2 size in cm

def R2D(value):                     # radians to degrees
    return value * 180 / math.pi

def cm2inch(value):                 # inch to cm
    return value/2.54

# plotVehicleStates(simTime, simData, figNo) plots the 6-DOF vehicle 
# position/attitude and velocities versus time in figure no. figNo
def plotVehicleStates(simTime, simData, figNo):
           
    # time vector
    t = simTime
        
    # state vectors
    x = simData[:,0]
    y = simData[:,1]
    z = simData[:,2]
    phi   = R2D(simData[:,3])    
    theta = R2D(simData[:,4])  
    psi   = R2D(simData[:,5])  
    u = simData[:,6]    
    v = simData[:,7]  
    w = simData[:,8]  
    p = R2D(simData[:,9])    
    q = R2D(simData[:,10])  
    r = R2D(simData[:,11]) 
   
    # speed
    U = np.sqrt( np.multiply(u,u) + np.multiply(v,v) + np.multiply(w,w) )
     
    beta_c  = R2D(np.arctan2(v,u))         # crab angle
    alpha_c = R2D(np.arctan2(w,u))         # flight path angle
    chi     = psi + beta_c                 # course angle
    
    # plots
    plt.figure(figNo,figsize=(cm2inch(figSize1[0]),cm2inch(figSize1[1])))
    plt.grid()

    plt.subplot(3, 3, 1)
    plt.plot(y, x)
    plt.legend(['North-East positions (m)'],fontsize=legendSize) 
    plt.grid()
    
    plt.subplot(3, 3, 2)
    plt.plot(t, z)
    plt.legend(['Depth (m)'],fontsize=legendSize) 
    plt.grid()
    
    plt.title('Vehicle states', fontsize=12)

    plt.subplot(3, 3, 3)
    plt.plot(t, phi, t, theta)    
    plt.legend(['Roll angle (deg)','Pitch angle (deg)'],fontsize=legendSize) 
    plt.grid()

    plt.subplot(3, 3, 4)
    plt.plot(t, U)
    plt.legend(['Speed (m/s)'],fontsize=legendSize)    
    plt.grid()

    plt.subplot(3, 3, 5)
    plt.plot(t, chi)    
    plt.legend(['Course angle (deg)'],fontsize=legendSize)    
    plt.grid()

    plt.subplot(3, 3, 6)
    plt.plot(t, beta_c, t, psi)    
    plt.legend(['Crab angle (deg)','Yaw angle (deg)'],fontsize=legendSize)
    plt.grid()
    
    plt.subplot(3, 3, 7)
    plt.plot(t, u, t, v, t, w)  
    plt.xlabel('Time (s)', fontsize=12)     
    plt.legend(['Surge velocity (m/s)','Sway velocity (m/s)',
                'Heave velocity (m/s)'],fontsize=legendSize)   
    plt.grid()

    plt.subplot(3, 3, 8)
    plt.plot(t, p, t, q, t, r)
    plt.xlabel('Time (s)', fontsize=12) 
    plt.legend(['Roll rate (deg/s)','Pitch rate (deg/s)',
                'Yaw rate (deg/s)'],fontsize=legendSize)
    plt.grid()

    plt.subplot(3, 3, 9)
    plt.plot(t, alpha_c, t, theta)
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Flight path angle (deg)','Pitch angle (deg)'],fontsize=legendSize)      
    plt.grid()

# plotControls(simTime, simData) plots the vehicle control inputs versus time
# in figure no. figNo
def plotControls(simTime, simData, vehicle, figNo):

    DOF = 6
    
    # time vector
    t = simTime
    
    plt.figure(figNo,figsize=(cm2inch(figSize2[0]),cm2inch(figSize2[1])))
    
    # col and rows needed to plot vehicle.dimU control inputs
    col = 2
    row = int( math.ceil(vehicle.dimU / col) )

    # plot the vehicle.dimU active control inputs
    for i in range(0,vehicle.dimU):
        
        u = simData[:,2*DOF+i]                  # control input

        if vehicle.controls[i].find('deg'):     # convert angles to deg
            u = R2D(u)

        plt.subplot(row, col, i+1)
        plt.plot(t, u)
        plt.legend([vehicle.controls[i]],fontsize=legendSize) 
        plt.grid()    
    
    