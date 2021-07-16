#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 15:07:02 2021

@author: thor
"""

from functions.plotTimeSeries import plotVehicleStates
from functions.mainLoop import simulate

# from classes.vehicle import *

# Global constants
sampleTime = 0.1
N = 100

# Intitial states
x = 0

# Main program
def main():
    
    [simTime, simData] = simulate(N, sampleTime, x)
    plotVehicleStates(simTime, simData)    
            
main()
