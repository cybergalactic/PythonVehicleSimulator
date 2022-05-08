#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
shipClarke83.py:  

   Class for a generic ship parametrized using the main dimensions L, B, and T.
   The ship model is based on the linear maneuvering coefficients by 
   Clarke (1983).  
       
   shipClarke83()                           
       Step input, rudder angle    
       
   shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)
        psi_d: desired yaw angle (deg)
        L: ship length (m)
        B: ship beam (m)
        T: ship draft (m)
        Cb: block coefficient (-)
        V_c: current speed (m/s)
        beta_c: current direction (deg)
        tau_X: surge force, pilot input (N)                    

Methods:
        
    [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime ) returns 
        nu[k+1] and u_actual[k+1] using Euler's method. The control input is:

        u_control = delta_r (rad) is for the ship rudder.

    u = headingAutopilot(eta,nu,sampleTime) 
        PID controller for automatic heading control based on pole placement.
       
    u = stepInput(t) generates rudder step inputs.   
       
References: 

    D. Clarke, P. Gedling and G. Hine (1983). The Application of Manoeuvring 
        Criteria in Hull Design using Linear Theory. Transactions of the Royal 
        Institution of Naval Architects (RINA), 125, pp. 45-68.
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
         Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
from python_vehicle_simulator.lib.control import PIDpolePlacement
from python_vehicle_simulator.lib.models import clarke83

# Class Vehicle
class shipClarke83:
    """
    shipClarke83()
        Rudder angle step inputs
    shipClarke83('headingAutopilot', psi_d, L, B, T, Cb, V_c, beta_c, tau_X)
        Heading autopilot
        
    Inputs:
        psi_d: desired yaw angle (deg)
        L: ship length (m)
        B: ship beam (m)
        T: ship draft (m)
        Cb: block coefficient (-)
        V_c: current speed (m/s)
        beta_c: current direction (deg)
        tau_X: surge force, pilot input (N) 
    """

    def __init__(
        self,
        controlSystem="stepInput",
        r = 0,
        L = 50.0,
        B = 7.0,
        T = 5.0,
        Cb = 0.7,
        V_current = 0,
        beta_current = 0,
        tau_X = 1e5,
    ):
        
        # Constants
        D2R = math.pi / 180     # deg2rad
        self.rho = 1026         # density of water (kg/m^3)

        if controlSystem == "headingAutopilot":
            self.controlDescription = "Heading autopilot, psi_d = " + str(r) + " deg"

        else:
            self.controlDescription = "Step input for delta_r"
            controlSystem = "stepInput"

        self.ref = r
        self.V_c = V_current
        self.beta_c = beta_current * D2R
        self.controlMode = controlSystem

        # Initialize the ship model
        self.name = "Linear ship maneuvering model (see 'shipClarke83.py' for more details)"
        self.L = L  # length (m)
        self.B = B  # beam (m)
        self.T = T  # Draft (m)
        self.Cb = Cb  # block coefficient
        self.Lambda = 0.7  # rudder aspect ratio:  Lambda = b**2 / AR
        self.tau_X = tau_X  # surge force (N), pilot input
        self.deltaMax = 30  # max rudder angle (deg)
        self.T_delta = 1.0  # rudder time constants (s)
        self.nu = np.array([2, 0, 0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0], float)  # control input vector

        if self.L > 100:
            self.R66 = 0.27 * self.L  # approx. radius of gyration in yaw (m)
        else:
            self.R66 = 0.25 * self.L

        self.controls = ["Rudder angle (deg)"]
        self.dimU = len(self.controls)

        # Heading autopilot
        self.e_int = 0  # integral state
        self.wn = 0.3   # PID pole placement
        self.zeta = 1

        # Reference model
        self.r_max = 1.0 * D2R  # maximum yaw rate
        self.psi_d = 0          # angle, angular rate and angular acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5  # desired natural frequency in yaw
        self.zeta_d = 1  # desired relative damping ratio in yaw

        # controller parameters m, d and k
        U0 = 3  # cruise speed
        [M, N] = clarke83(U0, self.L, self.B, self.T, self.Cb, self.R66, 0, self.L)
        self.m_PID = M[2][2]
        self.d_PID = N[2][2]
        self.k_PID = 0

        # Rudder yaw moment coefficient (Fossen 2021, Chapter 9.5.1)
        b = 0.7 * self.T  # rudder height
        AR = b ** 2 / self.Lambda  # aspect ratio: Lamdba = b**2/AR
        CN = 6.13 * self.Lambda / (self.Lambda + 2.25)  # normal coefficient
        t_R = 1 - 0.28 * self.Cb - 0.55
        a_H = 0.4
        x_R = -0.45 * self.L
        x_H = -1.0 * self.L

        # tau_N = Yd * delta
        self.Nd = -0.25 * (x_R + a_H * x_H) * self.rho * U0 ** 2 * AR * CN

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
        the ship equations of motion using Euler's method.
        """

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity vector
        nu_r = nu - nu_c  # relative velocity vector

        U_r = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2)  # relative speed

        # Rudder command and actual rudder angle
        delta_c = u_control[0]
        delta = u_actual[0]

        # Rudder forces and moment (Fossen 2021, Chapter 9.5.1)
        b = 0.7 * self.T  # rudder height
        AR = b ** 2 / self.Lambda  # aspect ratio: Lamdba = b**2/AR
        CN = 6.13 * self.Lambda / (self.Lambda + 2.25)  # normal coefficient
        t_R = 1 - 0.28 * self.Cb - 0.55
        a_H = 0.4
        x_R = -0.45 * self.L
        x_H = -1.0 * self.L

        Xdd = -0.5 * (1 - t_R) * self.rho * U_r ** 2 * AR * CN
        Yd = -0.25 * (1 + a_H) * self.rho * U_r ** 2 * AR * CN
        Nd = -0.25 * (x_R + a_H * x_H) * self.rho * U_r ** 2 * AR * CN

        # Control forces and moment
        delta_R = -delta  # physical rudder angle (rad)
        T = self.tau_X  # thrust (N)
        t_deduction = 0.1  # thrust deduction number
        tau1 = (1 - t_deduction) * T - Xdd * math.sin(delta_R) ** 2
        tau2 = -Yd * math.sin(2 * delta_R)
        tau6 = -Nd * math.sin(2 * delta_R)
        tau = np.array([tau1, tau2, tau6], float)

        # Linear maneuvering model
        T_surge = self.L  # approx. time constant in surge (s)
        xg = 0  # approx. x-coordinate, CG (m)

        # 3-DOF ship model
        [M, N] = clarke83(U_r, self.L, self.B, self.T, self.Cb, self.R66, xg, T_surge)
        Minv = np.linalg.inv(M)
        nu3 = np.array([nu_r[0], nu_r[1], nu_r[5]])
        nu3_dot = np.matmul(Minv, tau - np.matmul(N, nu3))

        # 6-DOF ship model
        nu_dot = np.array([nu3_dot[0], nu3_dot[1], 0, 0, 0, nu3_dot[2]])

        # Rudder angle saturation
        if abs(delta) >= self.deltaMax * math.pi / 180:
            delta = np.sign(delta) * self.deltaMax * math.pi / 180

        # Rudder dynamics
        delta_dot = (delta_c - delta) / self.T_delta

        # Forward Euler integration [k+1]
        nu = nu + sampleTime * nu_dot
        delta = delta + sampleTime * delta_dot

        u_actual = np.array([delta], float)

        return nu, u_actual

    def stepInput(self, t):
        """
        delta_c = stepInput(t) generates rudder step inputs.
        """
        delta_c = 10 * (math.pi / 180)
        if t > 50:
            delta_c = 0

        u_control = np.array([delta_c], float)

        return u_control

    def headingAutopilot(self, eta, nu, sampleTime):
        """
        delta_c = headingAutopilot(eta,nu,sampleTime) is a PID controller
        for automatic heading control based on pole placement.

        tau_N = m * a_d + d * r_d
              - Kp * ( ssa( psi-psi_d ) + Td * (r - r_d) + (1/Ti) * e_int )
        """
        psi = eta[5]  # yaw angle
        r = nu[5]  # yaw rate
        e_psi = psi - self.psi_d  # yaw angle tracking error
        e_r = r - self.r_d  # yaw rate tracking error
        psi_ref = self.ref * math.pi / 180  # yaw angle setpoint

        wn = self.wn  # PID natural frequency
        zeta = self.zeta  # PID natural relative damping factor
        wn_d = self.wn_d  # reference model natural frequency
        zeta_d = self.zeta_d  # reference model relative damping factor

        m = self.m_PID
        d = self.d_PID
        k = 0

        # PID feedback controller with 3rd-order reference model
        [tau_N, self.e_int, self.psi_d, self.r_d, self.a_d] = PIDpolePlacement(
            self.e_int,
            e_psi,
            e_r,
            self.psi_d,
            self.r_d,
            self.a_d,
            m,
            d,
            k,
            wn_d,
            zeta_d,
            wn,
            zeta,
            psi_ref,
            self.r_max,
            sampleTime,
        )

        # Control allocation: tau_N = Yd * delta
        delta_c = tau_N / self.Nd  # rudder command

        u_control = np.array([delta_c], float)

        return u_control
