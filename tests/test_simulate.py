#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The pytest for 'test_simulate.py' can be run in the termonal using:
1) cd <path of the Python Vehicle Simulator installation>
2) pytest -k simulate -v
"""  

import matplotlib.pyplot as plt
from python_vehicle_simulator.lib import *
import python_vehicle_simulator.vehicles as vehicles
import pytest

sampleTime = 0.02                   # sample time
N = 10000                           # number of samples
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'safari'  

@pytest.fixture
def vehicle():
    yield vehicles.tanker('headingAutopilot',-20,0.5,150,20,80)

@pytest.fixture
def simulation(vehicle):
    yield simulate(N, sampleTime, vehicle)


def test_simulate(simulation):
    pass
    
def test_plot_vehicle_states(simulation):
    simTime, simData = simulation
    plotVehicleStates(simTime, simData, 1)
    #plt.show()

def test_plot_controls(simulation, vehicle):
    simTime, simData = simulation
    plotControls(simTime, simData, vehicle, 2)
    #plt.show()

def test_3D_plot(simulation, vehicle):
    simTime, simData = simulation
    plot3D(simData,numDataPoints,FPS,filename,3)
    #plt.show()
    
def test_DSRV():
    vehicle = vehicles.DSRV()
    simulate(N, sampleTime, vehicle)

def test_otter():
    vehicle = vehicles.otter()
    simulate(N, sampleTime, vehicle)

def test_ROVzefakkel():
    vehicle = vehicles.ROVzefakkel()
    simulate(N, sampleTime, vehicle)

def test_semisub():
    vehicle = vehicles.semisub()
    simulate(N, sampleTime, vehicle)

def test_shipClarke83():
    vehicle = vehicles.shipClarke83()
    simulate(N, sampleTime, vehicle)

def test_supply():
    vehicle = vehicles.supply()
    simulate(N, sampleTime, vehicle)

def test_frigate():
    vehicle = vehicles.frigate()
    simulate(N, sampleTime, vehicle)

def test_tanker():
    vehicle = vehicles.tanker()
    simulate(N, sampleTime, vehicle)
    
def test_remus100():
    vehicle = vehicles.remus100()
    simulate(N, sampleTime, vehicle)    
