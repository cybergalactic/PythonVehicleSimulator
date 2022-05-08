#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DSRV.py:  

    Class for the Naval Postgraduate School deep submergence rescue vehicle
    (DSRV). The length of the vehicle is L = 5.0 m and the state vector is 
    nu  = [ 0 0 w 0 q 0]' where w is the heave velocity (m/s) and q is the
    pitch rate (rad/s).  The constructors are:
        
    DSRV()                      
        Step input, rudder angel
        
    DSRV('deptAutopilot',z_d)  
        z_d: desired depth (m)
        
Methods:   
        
    [nu, u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) returns
         nu[k+1] and u_actual[k+1] using Euler's method. The control input is:
       
         u_control = delta_s (rad):  DSRV stern plane.

    u = depthAutopilot(eta,nu,sampleTime) 
        PID controller for automatic depth control based on pole placement.
       
    u = stepInput(t) generates stern plane step inputs.   
       
References: 
    
    A. J. Healey (1992). Marine Vehicle Dynamics Lecture Notes and 
        Problem Sets, Naval Postgraduate School (NPS), Monterey, CA.
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
        Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
from python_vehicle_simulator.lib.control import PIDpolePlacement

# Class Vehicle
class DSRV:
    """
    DSRV()                      Step input, rudder angle
    DSRV('deptAutopilot',z_d)   Depth autopilot
    
    Inputs:
        z_d: desired depth, positive downwards (m)
    """

    def __init__(self, controlSystem="stepInput", r=0):

        if controlSystem == "depthAutopilot":
            self.controlDescription = "Depth autopilot, z_d = " + str(r) + " m"

        else:
            self.controlDescription = "Step input for delta_s"
            controlSystem = "stepInput"

        self.ref = r
        self.controlMode = controlSystem

        # Initialize the DSRV model
        self.name = "DSRV (see 'DSRV.py' for more details)"
        self.L = 5.0            # Length
        self.deltaMax = 20      # max stern plane angle (deg)
        self.T_delta = 1.0      # rudder time constants (s)
        self.U0 = 4.11          # cruise speed: 4.11 m/s = 8 knots
        self.W0 = 0
        self.nu = np.array([self.U0, 0, self.W0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0], float)  # control input vector

        self.controls = ["Stern plane (deg)"]
        self.dimU = len(self.controls)

        # Non-dimensional mass matrix
        Iy = 0.001925
        m = 0.036391
        Mqdot = -0.001573
        Zqdot = -0.000130
        Mwdot = -0.000146
        Zwdot = -0.031545

        self.m11 = m - Zwdot
        self.m12 = -Zqdot
        self.m22 = Iy - Mqdot
        self.m21 = -Mwdot

        self.detM = self.m11 * self.m22 - self.m12 * self.m21

        #  Non-dimensional hydrodynamic derivatives
        self.Mq = -0.01131
        self.Zq = -0.017455
        self.Mw = 0.011175
        self.Zw = -0.043938
        self.Mdelta = -0.012797
        self.Zdelta = 0.027695

        # Depth autopilot
        self.e_int = 0.0  # integral state, initial value
        self.wn = 1  # PID pole placement parameters
        self.zeta = 1

        # Reference model
        self.w_max = 1  # maximum heave velocity
        self.z_d = 0  # position, velocity and acc. states
        self.w_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5
        self.zeta_d = 1


    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu, u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        integrates the DSRV equations of motion using Euler's method.
        """

        # States and inputs
        delta_c = u_control[0]
        delta = u_actual[0]
        w = nu[2]
        q = nu[4]
        theta = eta[4]

        # Speed
        U = math.sqrt(self.U0 ** 2 + (self.W0 + w) ** 2)

        # Speed dependent pitch moment
        Mtheta = -0.156276 / U ** 2

        # Rudder angle saturation
        if abs(delta) >= self.deltaMax * math.pi / 180:
            delta = np.sign(delta) * self.deltaMax * math.pi / 180

        # Forces and moments
        Z = self.Zq * q + self.Zw * w + self.Zdelta * delta
        M = self.Mq * q + self.Mw * w + Mtheta * theta + self.Mdelta * delta

        # State derivatives (with dimension)
        nu_dot = np.zeros(6)
        nu_dot[2] = (self.m22 * Z - self.m12 * M) / self.detM
        nu_dot[4] = (-self.m21 * Z + self.m11 * M) / self.detM

        # stern plane dynamics
        delta_dot = (delta_c - delta) / self.T_delta  # rudder dynamics

        # Forward Euler integration [k+1]
        nu = nu + sampleTime * nu_dot
        delta = delta + sampleTime * delta_dot

        # Cruise speed (constant)
        nu[0] = self.U0

        u_actual = np.array([delta], float)

        return nu, u_actual


    def stepInput(self, t):
        """
        delta_c = stepInput(t) generates stern plane step inputs.
        """
        delta_c = 20 * (math.pi / 180)
        if t > 30:
            delta_c = 10 * (math.pi / 180)
        if t > 50:
            delta_c = 0

        u_control = np.array([delta_c], float)

        return u_control


    def depthAutopilot(self, eta, nu, sampleTime):
        """
        delta_c = depthAutopilot(eta,nu,sampleTime) is a PID controller for
        automatic depth control based on pole placement.
        """
        z = eta[2]              # heave position
        w = nu[2]               # heave velocity
        e_z = z - self.z_d      # heave position tracking error
        e_w = w - self.w_d      # heave velocity tracking error
        r = self.ref            # heave setpoint

        wn = self.wn            # PID natural frequency
        zeta = self.zeta        # PID natural relative damping factor
        wn_d = self.wn_d        # reference model natural frequency
        zeta_d = self.zeta_d    # reference model relative damping factor

        m = self.m11            # mass in heave including added mass
        d = 0
        k = 0

        # PID feedback controller with 3rd-order reference model
        [delta_c, self.e_int, self.z_d, self.w_d, self.a_d] = PIDpolePlacement(
            self.e_int,
            e_z,
            e_w,
            self.z_d,
            self.w_d,
            self.a_d,
            m,
            d,
            k,
            wn_d,
            zeta_d,
            wn,
            zeta,
            r,
            self.w_max,
            sampleTime,
        )

        u_control = np.array([delta_c], float)

        return u_control
