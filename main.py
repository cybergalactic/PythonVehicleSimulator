# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley    
    
Author:     Thor I. Fossen
"""
import matplotlib.pyplot as plt
from functions import plotVehicleStates,plotControls,simulate
from vehicles import *

# Simulation parameters: sample time and number of samples
sampleTime = 0.02
N = 10000 

""" 
DSRV('depthAutopilot',z_d)                                       
frigate('headingAutopilot',U,psi_d)
otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
ROVzefakkel('headingAutopilot',U,psi_d)                          
semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                      
shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                      

Call constructors without arguments to test step inputs, e.g. DSRV(), otter(), etc. 
"""

# vehicle1 = DSRV('depthAutopilot',60.0) 
vehicle1 = otter('headingAutopilot',100.0,0.3,-30.0,200.0) 
# vehicle2 = ROVzefakkel('headingAutopilot',3.0,100.0)
vehicle2 = frigate('headingAutopilot',10.0,100.0)
# vehicle2 = shipClarke83('headingAutopilot',-20.0,70,8,6,0.7,0.5,-10.0,1e5)
# vehicle2 = supply('DPcontrol',4.0,4.0,100.0,0.5,-20.0)
# vehicle2 = semisub('DPcontrol',10.0,2.0,20.0,0.5,-20.0)

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
