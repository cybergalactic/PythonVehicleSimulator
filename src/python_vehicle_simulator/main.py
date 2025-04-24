#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK. 
URL: https://www.fossen.biz/wiley  
    
Author:     Thor I. Fossen
"""
import os
import sys 
import webbrowser
import matplotlib.pyplot as plt
from python_vehicle_simulator.vehicles import (
    DSRV, frigate, otter, ROVzefakkel, semisub, shipClarke83, supply, tanker, 
    remus100, torpedo
)
from python_vehicle_simulator.lib import (
    printSimInfo, printVehicleinfo, simulate, plotVehicleStates, plotControls, 
    plot3D
)

### Simulation parameters ###
sampleTime = 0.02                   # sample time [seconds]
N = 10000                           # number of samples

# 3D plot and animation settings where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'safari'                  # browser for visualization of animated GIF

### Vehicle constructors ###
"""
Vehicle constructors:
  DSRV('depthAutopilot', z_d)                                        
  frigate('headingAutopilot', U, psi_d)
  otter('headingAutopilot', psi_d, V_c, beta_c, tau_X)                  
  ROVzefakkel('headingAutopilot', U, psi_d)                          
  semisub('DPcontrol', x_d, y_d, psi_d, V_c, beta_c)                      
  shipClarke83('headingAutopilot', psi_d, L, B, T, Cb, V_c, beta_c, tau_X)  
  supply('DPcontrol', x_d, y_d, psi_d, V_c, beta_c)      
  tanker('headingAutopilot', psi_d, V_c, beta_c, depth)    
  remus100('depthHeadingAutopilot', z_d, psi_d, V_c, beta_c)             
  torpedo('depthHeadingAutopilot', z_d, psi_d, V_c, beta_c)             

Call constructors without arguments to test step inputs, e.g. DSRV(), otter(), etc.
"""

### Main program ###
def main():
    printSimInfo() 

    no = input("Please enter a vehicle no.: ")   

    vehicleOptions = {
        '1': lambda: DSRV('depthAutopilot', 60.0),
        '2': lambda: frigate('headingAutopilot', 10.0, 100.0),
        '3': lambda: otter('headingAutopilot', 100.0, 0.3, -30.0, 200.0),
        '4': lambda: ROVzefakkel('headingAutopilot', 3.0, 100.0),
        '5': lambda: semisub('DPcontrol', 10.0, 10.0, 40.0, 0.5, 190.0),
        '6': lambda: shipClarke83('headingAutopilot', -20.0, 70, 8, 6, 0.7, 0.5, 10.0, 1e5),
        '7': lambda: supply('DPcontrol', 4.0, 4.0, 50.0, 0.5, 20.0),
        '8': lambda: tanker('headingAutopilot', -20, 0.5, 150, 20, 80),
        '9': lambda: remus100('depthHeadingAutopilot', 30, 50, 1525, 0.5, 170),
        '10': lambda: torpedo('depthHeadingAutopilot', 30, 50, 1525, 0.5, 170),
    }

    if no in vehicleOptions:
        vehicle = vehicleOptions[no]()
        printVehicleinfo(vehicle, sampleTime, N)
    else:
        print('Error: Not a valid simulator option')
        sys.exit()

    # Main simulation loop 
    [simTime, simData] = simulate(N, sampleTime, vehicle)
    
    # 3D plots and animation
    plotVehicleStates(simTime, simData, 1)                    
    plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)   
    
    """ Uncomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    
    plt.show()
    plt.close()

if __name__ == "__main__":
    main()
    
