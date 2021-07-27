# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley    
    
Author:     Thor I. Fossen
Date:       25 July 2021
"""
import matplotlib.pyplot as plt
from functions import plotVehicleStates,plotControls,simulate
from vehicles import DSRV,otter,shipClarke83

""" 
DSRV()                                    Stern plan, step input
DSRV('depthAutopilot',z_d)                PID depth autopilot

otter()                                   Propeller revolutions, step inputs
otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)   PID heading autopilot

shipClarke83()                            Rudder, step input
shipClarke83('headingAutopilot',L,B,T,psi_d,L,B,T,Cb,V_current,beta_c,tau_X)
   PID heading autopilot
"""

# vehicle1 = DSRV('depthAutopilot',60.0) 
vehicle1 = shipClarke83() 

vehicle2 = otter('headingAutopilot',100.0,0,0.1,120) 

# Simulation parameters: sample time and number of samples
sampleTime = 0.02
N = 3000            

# Main simulation loop 
def main():
    
    [simTime1, simData1] = simulate(N, sampleTime, vehicle1)   
    plotVehicleStates(simTime1, simData1, 1)                    
    plotControls(simTime1, simData1, vehicle1, 2)

    #[simTime2, simData2] = simulate(N, sampleTime, vehicle2)   
    #plotVehicleStates(simTime2, simData2, 3)    
    #plotControls(simTime2, simData2, vehicle2, 4)

    plt.show()

main()
