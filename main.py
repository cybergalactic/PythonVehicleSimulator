#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
    
Author:     Thor I. Fossen
Date:       18 July 2021
"""

from functions import plotVehicleStates, simulate
from vehicles import DSRV

# Choose vehicle from vehicles
vehicle = DSRV()

# Simulation parameters
sampleTime = 0.1    # sampling time used for numerical integration
N = 10              # number of samples

# Main program
print("The Python Vehicle Simulator:")
print("Simulation time: %.0f seconds" % (N * sampleTime) )
print("Vehicle:         %s (L = %s)" % (vehicle.name, vehicle.L))
print("Control system:  Heading autopilot")

# Main simulation loop
def main():
        
    [simTime, simData] = simulate(N, sampleTime, vehicle)   
    plotVehicleStates(simTime, simData)    
            
main()
