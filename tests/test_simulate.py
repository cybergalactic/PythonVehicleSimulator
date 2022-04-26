import matplotlib.pyplot as plt
from python_vehicle_simulator import plotVehicleStates, plotControls, simulate
import python_vehicle_simulator.vehicles as vehicles
import pytest
import matplotlib.pyplot as plt

# Simulation parameters: sample time and number of samples
sampleTime = 0.02
N = 10000

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
    plotControls(simTime, simData, vehicle, 4)
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

@pytest.mark.skip("ToDo: Fix this one.")
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