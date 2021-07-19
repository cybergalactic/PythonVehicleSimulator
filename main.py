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
import matplotlib.pyplot as plt
from functions import plotVehicleStates,plotControls,simulate
from vehicles import DSRV

#  DSRV('stepInput',15.0)          Stern plan step input
#  DSRV('depthAutopilot',30.0)     PID depth autopilot
vehicle1 = DSRV('depthAutopilot',30.0) 
vehicle2 = DSRV('stepInput',15.0) 

# Simulation parameters: sample time and number of samples
sampleTime = 0.1
N = 1000            

# Main simulation loop 
def main():
    
    [simTime1, simData1] = simulate(N, sampleTime, vehicle1)   
    plotVehicleStates(simTime1, simData1, 1)                    
    plotControls(simTime1, simData1, vehicle1, 2)

    [simTime2, simData2] = simulate(N, sampleTime, vehicle2)   
    plotVehicleStates(simTime2, simData2, 3)    
    plotControls(simTime2, simData2, vehicle2, 4)

    plt.show()

main()
