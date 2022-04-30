#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley    
    
Author:     Thor I. Fossen
"""
import os
import webbrowser
import matplotlib.pyplot as plt
from python_vehicle_simulator.vehicles import *
from python_vehicle_simulator import *

# Simulation parameters: 
sampleTime = 0.02                   # sample time
N = 10000                           # number of samples

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'safari'                  # browser for visualization of animated GIF

"""
DSRV('depthAutopilot',z_d)                                       
frigate('headingAutopilot',U,psi_d)
otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
ROVzefakkel('headingAutopilot',U,psi_d)                          
semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                      
shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)      
tanker('headingAutopilot',psi_d,V_current,beta_c,depth)               

Call constructors without arguments to test step inputs, e.g. DSRV(), otter(), etc. 
"""

vehicle = DSRV('depthAutopilot',60.0) 
# vehicle = otter('headingAutopilot',100.0,0.3,-30.0,200.0) 
# vehicle = ROVzefakkel('headingAutopilot',3.0,100.0)
# vehicle = frigate('headingAutopilot',10.0,100.0)
# vehicle = tanker('headingAutopilot',-20,0.5,150,20,80)
# vehicle = shipClarke83('headingAutopilot',-20.0,70,8,6,0.7,0.5,-10.0,1e5)
# vehicle = supply('DPcontrol',4.0,4.0,100.0,0.5,-20.0)
# vehicle = semisub('DPcontrol',10.0,2.0,20.0,0.5,-20.0)

def main():    # Main simulation loop 
    
    [simTime, simData] = simulate(N, sampleTime, vehicle)
    plotVehicleStates(simTime, simData, 1)                    
    plotControls(simTime, simData, vehicle, 2)
    plot3D(simData,numDataPoints,FPS,filename,3)   
    
    """ Ucomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    
    plt.show()
    plt.close()

main()
