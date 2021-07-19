#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley    
    
Author:     Thor I. Fossen
Date:       18 July 2021
"""
from functions import plotVehicleStates,plotControls,simulate,simInfo
from vehicles import DSRV

# Choose a vehicle: {DSRV, ship}
# type help(DSRV) etc. to see the control system options 

#vehicle = DSRV()
#vehicle = DSRV('stepInput',15)   
vehicle = DSRV('depthAutopilot',30.0)   

# Simulation parameters: sample time and number of samples
sampleTime = 0.1      
N = 1000            

simInfo(vehicle)            # print main simulator info

# Main simulation loop
def main():
        
    [simTime, simData] = simulate(N, sampleTime, vehicle)   
    plotVehicleStates(simTime, simData)    
    plotControls(simTime, simData, vehicle)
            
main()
