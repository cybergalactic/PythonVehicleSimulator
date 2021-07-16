#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 16:48:53 2021

@author: thor
"""

# Function 
def plotVehicleStates(simTime, simData):
    
    import matplotlib.pyplot as plt
    
    plt.style.use('seaborn')
    fig, axs = plt.subplots(1,2)
    
    axs[0].plot(simTime, simData[:,0],simTime, simData[:,1])
    axs[0].set_title('State x', fontsize=24)
    axs[0].set_xlabel('Time (s)', fontsize=12)  
    
    axs[1].plot(simTime, simData[:,0],'r')
    axs[1].set_title('State x', fontsize=24)    
    axs[1].set_xlabel('Time (s)', fontsize=12)   
    
    plt.show()
    