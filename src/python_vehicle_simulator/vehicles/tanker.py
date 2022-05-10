#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tanker.py:  

   Class for a large tanker, length L = 304.8 m and draft T = 18.46 m. The 
   input variable 'depth' can be used to simulate shallow water effects. 
       
   tanker()                           
       Step input, rudder angle     

    tanker('headingAutopilot',psi_d,V_current,beta_c,depth,rpm)
        psi_d:  desired yaw angle (deg)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)
        depth:  the water depth must be larger than the draft T = 18.46 m 
        rpm:    shaft speed, nominal propeller rpm = 80
                   
Methods:
        
    [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime ) returns 
        nu[k+1] and u_actual[k+1] using Euler's method. The control input is:

        u_control = delta_r (rad) is for the ship rudder.

    u = headingAutopilot(eta,nu,sampleTime) 
        PID controller for automatic heading control based on pole placement.
       
    u = stepInput(t) generates rudder step inputs.   
       
References: 
    
    W. B. Van Berlekom and T. A. Goddard (1972). Maneuvering of Large Tankers,
        Transaction of SNAME, Vol. 80, pp. 264-298.
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
        Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
import sys
from python_vehicle_simulator.lib.control import PIDpolePlacement

# Class Vehicle
class tanker:
    """
    tanker()
        Rudder angle step inputs
    tanker('headingAutopilot',psi_d,V_current,beta_c,depth,rpm)
        Heading autopilot
        
    Inputs:
        psi_d:  desired yaw angle (deg)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)
        depth:  the water depth must be larger than the draft T = 18.46 m 
        rpm:    shaft speed, nominal propeller rpm = 80       
    """

    def __init__(
        self,
        controlSystem="stepInput",
        r = 0,
        V_current = 0,
        beta_current = 0,
        depth = 20.0,
        rpm = 80.0,
    ):

        # Constants
        self.D2R = math.pi / 180     # deg2rad
        
        if controlSystem == "headingAutopilot":
            self.controlDescription = ("Heading autopilot, psi_d = " 
            + str(r) + " deg")
        else:
            self.controlDescription = "Step input for delta_r"
            controlSystem = "stepInput"

        self.ref = r
        self.V_c = V_current
        self.beta_c = beta_current * self.D2R
        self.waterDdepth = depth
        self.n_c = rpm
        self.controlMode = controlSystem

        # Initialize the ship model
        self.name = "Tanker (see 'tanker.py' for more details)"
        self.L = 304.8      # length (m)
        self.T = 18.46      # draft (m)
        self.deltaMax = 30  # max rudder angle (deg)
        self.DdeltaMax = 5  # max rudder rate (deg/s)
        self.nMax = 90.0    # max propeller shaft speed (rpm)

        if rpm < 10.0 or rpm > self.nMax:
            sys.exit("The RPM value should be in the interval 10-90")

        if depth < self.T:
            sys.exit("The water depth must be larger than the draft T = 18.46 m")

        self.nu = np.array([4.8, 0, 0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0], float)  # control input vector

        self.controls = ["Rudder angle (deg)"]
        self.dimU = len(self.controls)

        # Heading autopilot
        self.e_int = 0          # integral state
        self.wn = 0.15           # PID pole placement
        self.zeta = 0.8

        # Reference model
        self.r_max = 1.0 * math.pi / 180  # maximum yaw rate
        self.psi_d = 0  # angle, angular rate and angular acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5     # desired natural frequency in yaw
        self.zeta_d = 1.0           # desired relative damping ratio in yaw


    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
        the ship equations of motion using Euler's method.
        """
        h = self.waterDdepth  # water depth (m)
        L = self.L  # length (m)
        T = self.T  # draft (m)

        # States and controls
        delta_c = u_control[0]  # autopilot rudder command
        delta = u_actual[0]     # actual rudder angle (rad)
        n = self.n_c / 60.0     # propeller shaft speed (rps)
        # u = nu[0]
        # v = nu[1]
        r = nu[5]

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity vector
        nu_r = nu - nu_c  # relative velocity vector

        u_r = nu_r[0]
        v_r = nu_r[1]

        try:
            beta = v_r / u_r  # sideslip angle (rad)

        except ZeroDivisionError:
            print("The sideslip angle is not defined for u_r = u - u_c = 0")

        # Parameters and hydrodynamic derivatives
        t = 0.22

        cun = 0.605
        cnn = 38.2

        Tuu = -0.00695
        Tun = -0.00063
        Tnn = 0.0000354

        m11 = 1.050     # 1 - Xudot
        m22 = 2.020     # 1 - Yvdot
        m33 = 0.1232    # kz^2 - Nrdot

        d11 = 2.020     # 1 + Xvr
        d22 = -0.752    # Yur - 1
        d33 = -0.231    # Nur - xG

        Xuuz = -0.0061
        YT = 0.04
        NT = -0.02
        Xuu = -0.0377
        Yvv = -2.400
        Nvr = -0.300
        Xvv = 0.3
        Yuv = -1.205
        Nuv = -0.451
        Xudotz = -0.05
        Yvdotz = -0.387
        Nrdotz = -0.0045
        Xuuz = -0.0061
        Yurz = 0.182
        Nurz = -0.047
        Xvrz = 0.387
        Yvvz = -1.5
        Nvrz = -0.120
        Xccdd = -0.093
        Yuvz = 0.0
        Nuvz = -0.241
        Xccbd = 0.152
        Yccd = 0.208
        Nccd = -0.098
        Xvvzz = 0.0125
        Yccbbd = -2.16
        Nccbbd = 0.688
        Yccbbdz = -0.191
        Nccbbdz = 0.344

        # Shallow water effects
        z = T / (h - T)
        if z >= 0.8:
            Yuvz = -0.85 * (1.0 - 0.8 / z)

        # Forces and moment
        gT = 1 / L * Tuu * u_r ** 2 + Tun * u_r * n + L * Tnn * abs(n) * n
        c = math.sqrt(cun * u_r * n + cnn * n ** 2)

        gX = (
            1
            / L
            * (
                Xuu * u_r ** 2
                + L * d11 * v_r * r
                + Xvv * v_r ** 2
                + Xccdd * abs(c) * c * delta ** 2
                + Xccbd * abs(c) * c * beta * delta
                + L * gT * (1 - t)
                + Xuuz * u_r ** 2 * z
                + L * Xvrz * v_r * r * z
                + Xvvzz * v_r ** 2 * z ** 2
            )
        )

        gY = (
            1
            / L
            * (
                Yuv * u_r * v_r
                + Yvv * abs(v_r) * v_r
                + Yccd * abs(c) * c * delta
                + L * d22 * u_r * r
                + Yccbbd * abs(c) * c * abs(beta) * beta * abs(delta)
                + YT * gT * L
                + L * Yurz * u_r * r * z
                + Yuvz * u_r * v_r * z
                + Yvvz * abs(v_r) * v_r * z
                + Yccbbdz * abs(c) * c * abs(beta) * beta * abs(delta) * z
            )
        )

        gLN = (
            Nuv * u_r * v_r
            + L * Nvr * abs(v_r) * r
            + Nccd * abs(c) * c * delta
            + L * d33 * u_r * r
            + Nccbbd * abs(c) * c * abs(beta) * beta * abs(delta)
            + L * NT * gT
            + L * Nurz * u_r * r * z
            + Nuvz * u_r * v_r * z
            + L * Nvrz * abs(v_r) * r * z
            + Nccbbdz * abs(c) * c * abs(beta) * beta * abs(delta) * z
        )

        # Shallow water effects
        m11 = m11 - Xudotz * z
        m22 = m22 - Yvdotz * z
        m33 = m33 - Nrdotz * z

        # Dimensional state derivatives
        nu_dot = np.array([gX / m11, gY / m22, 0.0, 0.0, 0.0, gLN / (L ** 2 * m33)])

        # Rudder angle saturation and dynamics
        if abs(delta) >= self.deltaMax * math.pi / 180:
            delta = np.sign(delta) * self.deltaMax * math.pi / 180

        delta_dot = delta_c - delta
        if abs(delta_dot) >= self.DdeltaMax * math.pi / 180:
            delta_dot = np.sign(delta_dot) * self.DdeltaMax * math.pi / 180

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
        psi = eta[5]                        # yaw angle
        r = nu[5]                           # yaw rate
        e_psi = psi - self.psi_d            # yaw angle tracking error
        e_r = r - self.r_d                  # yaw rate tracking error
        psi_ref = self.ref * self.D2R       # yaw angle setpoint

        wn = self.wn            # PID natural frequency
        zeta = self.zeta        # PID natural relative damping factor
        wn_d = self.wn_d        # reference model natural frequency
        zeta_d = self.zeta_d    # reference model relative damping factor

        m = 500
        d = 0.0
        k = 0.0

        # PID feedback controller with 3rd-order reference model
        [delta_r, self.e_int, self.psi_d, self.r_d, self.a_d] = PIDpolePlacement(
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

        delta = -delta_r
        u_control = np.array([delta], float)

        return u_control
