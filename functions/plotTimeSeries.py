# -*- coding: utf-8 -*-
"""
Simulator plotting functions:

plotVehicleStates(simTime, simData, figNo) 
plotControls(simTime, simData))

Author:     Thor I. Fossen
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from functions.gnc import ssa 

legendSize = 10                     # legend size
figSize1 = [25, 13]                 # figure1 size in cm
figSize2 = [25, 13]                 # figure2 size in cm
dpiValue = 150                      # figure dpi value

def R2D(value):                     # radians to degrees
    return value * 180 / math.pi

def cm2inch(value):                 # inch to cm
    return value/2.54

# plotVehicleStates(simTime, simData, figNo) plots the 6-DOF vehicle 
# position/attitude and velocities versus time in figure no. figNo
def plotVehicleStates(simTime, simData, figNo):
           
    # Time vector
    t = simTime
        
    # State vectors
    x = simData[:,0]
    y = simData[:,1]
    z = simData[:,2]
    phi   = R2D(ssa(simData[:,3]))   
    theta = R2D(ssa(simData[:,4]))  
    psi   = R2D(ssa(simData[:,5]))  
    u = simData[:,6]    
    v = simData[:,7]  
    w = simData[:,8]  
    p = R2D(simData[:,9])    
    q = R2D(simData[:,10])  
    r = R2D(simData[:,11]) 
   
    # Speed
    U = np.sqrt( np.multiply(u,u) + np.multiply(v,v) + np.multiply(w,w) )
     
    beta_c  = R2D(ssa(np.arctan2(v,u)))          # crab angle, beta_c
    alpha_c = R2D(ssa(np.arctan2(w,u)))          # flight path angle 
    chi = R2D(ssa(simData[:,5]+np.arctan2(v,u))) # course angle, chi=psi+beta_c 

    # Plots
    plt.figure(figNo,figsize=(cm2inch(figSize1[0]),cm2inch(figSize1[1])),
               dpi=dpiValue)
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
    plt.plot(t, theta, t, alpha_c)
    plt.legend(['Pitch angle (deg)','Flight path angle (deg)'],
               fontsize=legendSize)      
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
    plt.plot(t, psi, t, beta_c)  
    plt.xlabel('Time (s)', fontsize=12)   
    plt.legend(['Yaw angle (deg)','Crab angle (deg)'],fontsize=legendSize)
    plt.grid()   

# plotControls(simTime, simData) plots the vehicle control inputs versus time
# in figure no. figNo
def plotControls(simTime, simData, vehicle, figNo):

    DOF = 6
    
    # Time vector
    t = simTime
    
    plt.figure(figNo,figsize=(cm2inch(figSize2[0]),cm2inch(figSize2[1])),
               dpi=dpiValue)
    
    # Columns and rows needed to plot vehicle.dimU control inputs
    col = 2
    row = int( math.ceil(vehicle.dimU / col) )

    # Plot the vehicle.dimU active control inputs
    for i in range(0,vehicle.dimU):
        
        u_control = simData[:,2*DOF+i]                # control input, commands
        u_actual  = simData[:,2*DOF+vehicle.dimU+i]   # actual control input
        
        if vehicle.controls[i].find("deg") != -1:     # convert angles to deg
            u_control = R2D(u_control)
            u_actual  = R2D(u_actual)

        plt.subplot(row, col, i+1)
        plt.plot(t, u_control, t, u_actual)
        plt.legend([
            vehicle.controls[i] + ', command', 
            vehicle.controls[i] + ', actual'
            ], fontsize=legendSize) 
        plt.xlabel('Time (s)', fontsize=12) 
        plt.grid()    

    
    