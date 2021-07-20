# Python Vehicle Simulator

The Python Vehicle Simulator is a suppleement to the Matlab MSS (Marine Systems Simulator) toolbox. It includes models for autonomous underwater vehicles (AUVs), unmanned surface vehicles (USVs), and ships. The vehicle models are based on the MSS vessel models located in MSS/VESSELS/ catalogue. Each vehicle is modelled as an object in Python and the vehicle class has methods for guidance, navigation and control. The main program:

    main.py  
    
is used to define multiple vehicle objects for real-time simulation. The python modules are located under the catalogues: 

    /vehicles/                 Vehicle classes and methods:  
        DSRV.py                     - Deep submergence rescue vehicle, L = 5.0 m
        otter.py                    - Otter USV, L = 1.2 m
    /functions/                Functions used my the main program:
        guidance.py                 - guidance functions
        control.py                  - feedback control systems
        kinematics.py               - kinematic functions
        mainLoop.py                 - main simulation loop
        plotTimeSeries.py           - plotting functions
        
In order to run the main program main.py the following modules must be installed:

    numpy       https://numpy.org/install/
    matplotlib  https://matplotlib.org/stable/users/installing.html

Author: Thor I. Fossen




    
    
    
