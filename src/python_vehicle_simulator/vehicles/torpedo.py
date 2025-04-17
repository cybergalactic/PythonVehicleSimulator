#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
torpedo.py:  

    Torpedo-shaped vehicle based on the REMUS 100 platform, with modified 
    fin-based control:
        
	- Includes a new finClass that allows fins to be placed at arbitrary 
        locations on the vehicle.
	- Computes the force vector based on the local relative water velocity.
	- Supports actuator dynamics for fin control based on input commands.

    The vehicle is controlled using four fins and a propeller. It has a length 
    of 1.6 m, a cylindrical body with a 19 cm diameter, and a total mass of 
    31.9 kg. The maximum speed is 2.5 m/s, achieved when the propeller runs at
    1525 RPM in still water (no current).
       
   torpedo()                           
       Step input, stern plane, rudder and propeller revolution     
   
    torpedo('depthHeadingAutopilot',z_d,psi_d,n_d,V_c,beta_c)
        z_d:    desired depth (m), positive downwards
        psi_d:  desired yaw angle (deg)
        n_d:    desired propeller revolution (rpm)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)

Methods:
        
    [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime ) returns 
        nu[k+1] and u_actual[k+1] using Euler's method. The control input is:

            u_control = [ delta_r_top   rudder angle (rad)
                         delta_r_bottom rudder angle (rad)
                         delta_s_star    stern plane angle (rad)
                         delta_s_port    stern plane angle (rad)
                         n          propeller revolution (rpm) ]

    u = depthHeadingAutopilot(eta,nu,sampleTime) 
        Simultaneously control of depth and heading using controllers of 
        PID and SMC ype. Propeller rpm is given as a step command.
       
    u = stepInput(t) generates tail rudder, stern planes and RPM step inputs.   
       
References: 
    
    B. Allen, W. S. Vorus and T. Prestero, "Propulsion system performance 
         enhancements on REMUS AUVs," OCEANS 2000 MTS/IEEE Conference and 
         Exhibition. Conference Proceedings, 2000, pp. 1869-1873 vol.3, 
         doi: 10.1109/OCEANS.2000.882209.    
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
         Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Braden Meyers
"""
import numpy as np
import math
import sys
from python_vehicle_simulator.lib.control import integralSMC
from python_vehicle_simulator.lib.gnc import crossFlowDrag,forceLiftDrag,Hmtrx,m2c,gvect,ssa
from python_vehicle_simulator.lib.actuator import fin, thruster

# Class Vehicle
class torpedo:
    """
    torpedo()
        Rudder angle, stern plane and propeller revolution step inputs
        
    torpedo('depthHeadingAutopilot',z_d,psi_d,n_d,V_c,beta_c) 
        Depth and heading autopilots
        
    Inputs:
        z_d:    desired depth, positive downwards (m)
        psi_d:  desired heading angle (deg)
        n_d:    desired propeller revolution (rpm)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)
    """

    def __init__(
        self,
        controlSystem="stepInput",
        r_z = 0,
        r_psi = 0,
        r_rpm = 0,
        V_current = 0,
        beta_current = 0,
    ):

        # Constants
        self.D2R = math.pi / 180        # deg2rad
        self.rho = 1026                 # density of water (kg/m^3)
        g = 9.81                        # acceleration of gravity (m/s^2)
        
        if controlSystem == "depthHeadingAutopilot":
            self.controlDescription = (
                "Depth and heading autopilots, z_d = "
                + str(r_z) 
                + ", psi_d = " 
                + str(r_psi) 
                + " deg"
                )

        else:
            self.controlDescription = (
                "Step inputs for stern planes, rudder and propeller")
            controlSystem = "stepInput"
            
        self.ref_z = r_z
        self.ref_psi = r_psi
        self.ref_n = r_rpm
        self.V_c = V_current
        self.beta_c = beta_current * self.D2R
        self.controlMode = controlSystem
        
        # Initialize the AUV model 
        self.name = (
            "Torpedo-shaped vehicle based on the REMUS 100 AUV (see 'torpedo100.py' for more details)")
        self.L = 1.6                # length (m)
        self.diam = 0.19            # cylinder diameter (m)
        
        self.nu = np.array([0, 0, 0, 0, 0, 0], float) # velocity vector
        self.controls = [
            "T Tail rudder (deg)",
            "B Tail rudder (deg)",
            "Star Stern plane (deg)",
            "Port Stern plane (deg)",
            "Propeller revolution (rpm)"
            ]
        self.dimU = len(self.controls) 
        self.u_actual = np.zeros(self.dimU, float)    # control input vector
        
        prop = thruster(self.rho)

        if r_rpm < 0.0 or r_rpm > prop.nMax:
            sys.exit("The RPM value should be in the interval 0-%s", (prop.nMax))
        
        if r_z > 100.0 or r_z < 0.0:
            sys.exit('desired depth must be between 0-100 m')    
        
        # Hydrodynamics (Fossen 2021, Section 8.4.2)    
        self.S = 0.7 * self.L * self.diam    # S = 70% of rectangle L * diam
        a = self.L/2                         # semi-axes
        b = self.diam/2                  
        self.r_bg = np.array([0, 0, 0.02], float)    # CG w.r.t. to the CO
        self.r_bb = np.array([0, 0, 0], float)       # CB w.r.t. to the CO

        # Parasitic drag coefficient CD_0, i.e. zero lift and alpha = 0
        # F_drag = 0.5 * rho * Cd * (pi * b^2)   
        # F_drag = 0.5 * rho * CD_0 * S
        Cd = 0.42                              # from Allen et al. (2000)
        self.CD_0 = Cd * math.pi * b**2 / self.S
        
        # Rigid-body mass matrix expressed in CO
        m = 4/3 * math.pi * self.rho * a * b**2     # mass of spheriod 
        Ix = (2/5) * m * b**2                       # moment of inertia
        Iy = (1/5) * m * (a**2 + b**2)
        Iz = Iy
        MRB_CG = np.diag([ m, m, m, Ix, Iy, Iz ])   # MRB expressed in the CG     
        H_rg = Hmtrx(self.r_bg)
        self.MRB = H_rg.T @ MRB_CG @ H_rg           # MRB expressed in the CO

        # Weight and buoyancy
        self.W = m * g
        self.B = self.W
        
        # Added moment of inertia in roll: A44 = r44 * Ix
        r44 = 0.3           
        MA_44 = r44 * Ix
        
        # Lamb's k-factors
        e = math.sqrt( 1-(b/a)**2 )
        alpha_0 = ( 2 * (1-e**2)/pow(e,3) ) * ( 0.5 * math.log( (1+e)/(1-e) ) - e )  
        beta_0  = 1/(e**2) - (1-e**2) / (2*pow(e,3)) * math.log( (1+e)/(1-e) )

        k1 = alpha_0 / (2 - alpha_0)
        k2 = beta_0  / (2 - beta_0)
        k_prime = pow(e,4) * (beta_0-alpha_0) / ( 
            (2-e**2) * ( 2*e**2 - (2-e**2) * (beta_0-alpha_0) ) )   

        # Added mass system matrix expressed in the CO
        self.MA = np.diag([ m*k1, m*k2, m*k2, MA_44, k_prime*Iy, k_prime*Iy ])
          
        # Mass matrix including added mass
        self.M = self.MRB + self.MA
        self.Minv = np.linalg.inv(self.M)

        # Natural frequencies in roll and pitch
        self.w_roll = math.sqrt( self.W * ( self.r_bg[2]-self.r_bb[2] ) / 
            self.M[3][3] )
        self.w_pitch = math.sqrt( self.W * ( self.r_bg[2]-self.r_bb[2] ) / 
            self.M[4][4] )
            
        S_fin = 0.00665;            # fin area 
        CL_delta_r = 0.5            # rudder lift coefficient
        CL_delta_s = 0.7            # stern-plane lift coefficient

        portSternFin = fin(S_fin, CL_delta_s, -a, c=0.1,angle=0, rho=self.rho)       
        bottomRudderFin = fin(S_fin, CL_delta_r, -a, c=0.1, angle=90, rho=self.rho)  
        starSternFin = fin(S_fin, CL_delta_s, -a, c=0.1, angle=180, rho=self.rho)      
        topRudderFin = fin(S_fin, CL_delta_r, -a, c=0.1, angle=270, rho=self.rho)  
        self.actuators = [topRudderFin, bottomRudderFin , starSternFin, portSternFin, prop]

        # Low-speed linear damping matrix parameters
        self.T_surge = 20           # time constant in surge (s)
        self.T_sway = 20            # time constant in sway (s)
        self.T_heave = self.T_sway  # equal for for a cylinder-shaped AUV
        self.zeta_roll = 0.3        # relative damping ratio in roll
        self.zeta_pitch = 0.8       # relative damping ratio in pitch
        self.T_yaw = 1              # time constant in yaw (s)
        
        # Feed forward gains (Nomoto gain parameters)
        self.K_nomoto = 5.0/20.0    # K_nomoto = r_max / delta_max
        self.T_nomoto = self.T_yaw  # Time constant in yaw
        
        # Heading autopilot reference model 
        self.psi_d = 0                    # position, velocity and acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = 0.1                   # desired natural frequency
        self.zeta_d = 1                   # desired realtive damping ratio 
        self.r_max = 5.0 * math.pi / 180  # maximum yaw rate 
        
        # Heading autopilot (Equation 16.479 in Fossen 2021)
        # sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
        # delta = (T_nomoto * r_r_dot + r_r - K_d * sigma 
        #       - K_sigma * (sigma/phi_b)) / K_nomoto
        self.lam = 0.1
        self.phi_b = 0.1       # boundary layer thickness
        self.K_d = 0.5         # PID gain
        self.K_sigma = 0.05    # SMC switching gain
        
        self.e_psi_int = 0     # yaw angle error integral state
        
        # Depth autopilot
        self.wn_d_z = 0.02     # desired natural frequency, reference model
        self.Kp_z = 0.1        # heave proportional gain, outer loop
        self.T_z = 100.0       # heave integral gain, outer loop
        self.Kp_theta = 5.0    # pitch PID controller     
        self.Kd_theta = 2.0  
        self.Ki_theta = 0.3
        self.K_w = 5.0         # optional heave velocity feedback gain
        
        self.z_int = 0         # heave position integral state
        self.z_d = 0           # desired position, LP filter initial state
        self.theta_int = 0     # pitch angle integral state
        

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
        the AUV equations of motion using Euler's method.
        """

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float) # current velocity 
        Dnu_c = np.array([nu[5]*v_c, -nu[5]*u_c, 0, 0, 0, 0],float) # derivative
        nu_r = nu - nu_c                               # relative velocity        
        alpha = math.atan2( nu_r[2], nu_r[0] )         # angle of attack 
        U_r = math.sqrt(nu_r[0]**2 + nu_r[1]**2 + nu_r[2]**2)  # relative speed

        # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
        CRB = m2c(self.MRB, nu_r)
        CA  = m2c(self.MA, nu_r)
        
        # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
        # rotational damping is missing. These terms are assumed to be zero
        CA[4][0] = 0     # Quadratic velocity terms due to pitching
        CA[0][4] = 0    
        CA[4][2] = 0
        CA[2][4] = 0
        CA[5][0] = 0     # Munk moment in yaw 
        CA[0][5] = 0
        CA[5][1] = 0
        CA[1][5] = 0
        
        C = CRB + CA

        # Dissipative forces and moments
        D = np.diag([
            self.M[0][0] / self.T_surge,
            self.M[1][1] / self.T_sway,
            self.M[2][2] / self.T_heave,
            self.M[3][3] * 2 * self.zeta_roll  * self.w_roll,
            self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
            self.M[5][5] / self.T_yaw
            ])
        
        # Linear surge and sway damping
        D[0][0] = D[0][0] * math.exp(-3*U_r) # vanish at high speed where quadratic
        D[1][1] = D[1][1] * math.exp(-3*U_r) # drag and lift forces dominates

        tau_liftdrag = forceLiftDrag(self.diam,self.S,self.CD_0,alpha,U_r)
        tau_crossflow = crossFlowDrag(self.L,self.diam,self.diam,nu_r)

        # Restoring forces and moments
        g = gvect(self.W,self.B,eta[4],eta[3],self.r_bg,self.r_bb)
        
        # General force vector
        tau = np.zeros(6,float)
        for i in range(self.dimU):
            tau += self.actuators[i].tau(nu_r, nu)
            u_actual[i] = self.actuators[i].actuate(sampleTime, u_control[i]) # Actuator Dynamics

        # AUV dynamics
        tau_sum = tau + tau_liftdrag + tau_crossflow - np.matmul(C+D,nu_r)  - g
        nu_dot = Dnu_c + np.matmul(self.Minv, tau_sum)
            

        # Forward Euler integration [k+1]
        nu += sampleTime * nu_dot

        return nu, u_actual


    def stepInput(self, t):
        """
        u_c = stepInput(t) generates step inputs.
                     
        Returns:
            
            u_control = [ delta_r   rudder angle (rad)
                         delta_s    stern plane angle (rad)
                         n          propeller revolution (rpm) ]
        """
        delta_r =  5 * self.D2R      # rudder angle (rad)
        delta_s = -5 * self.D2R      # stern angle (rad)
        n = 1525                     # propeller revolution (rpm)
        
        if t > 100:
            delta_r = 0
            
        if t > 50:
            delta_s = 0     

        u_control = np.array([ delta_r, -delta_r, -delta_s, delta_s, n], float)

        return u_control
    
    
    def depthHeadingAutopilot(self, eta, nu, sampleTime):
        """
        [delta_r, delta_s, n] = depthHeadingAutopilot(eta,nu,sampleTime) 
        simultaneously control the heading and depth of the AUV using control
        laws of PID type. Propeller rpm is given as a step command.
        
        Returns:
            
            u_control = [ delta_r   rudder angle (rad)
                         delta_s    stern plane angle (rad)
                         n          propeller revolution (rpm) ]
            
        """
        z = eta[2]                  # heave position (depth)
        theta = eta[4]              # pitch angle
        psi = eta[5]                # yaw angle
        w = nu[2]                   # heave velocity
        q = nu[4]                   # pitch rate
        r = nu[5]                   # yaw rate
        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        z_ref = self.ref_z          # heave position (depth) setpoint
        psi_ref = self.ref_psi * self.D2R   # yaw angle setpoint
        
        #######################################################################
        # Propeller command
        #######################################################################
        n = self.ref_n 
        
        #######################################################################            
        # Depth autopilot (succesive loop closure)
        #######################################################################
        # LP filtered desired depth command
        self.z_d  = math.exp( -sampleTime * self.wn_d_z ) * self.z_d \
            + ( 1 - math.exp( -sampleTime * self.wn_d_z) ) * z_ref  
            
        # PI controller    
        theta_d = self.Kp_z * ( (z - self.z_d) + (1/self.T_z) * self.z_int )
        delta_s = -self.Kp_theta * ssa( theta - theta_d ) - self.Kd_theta * q \
            - self.Ki_theta * self.theta_int - self.K_w * w

        # Euler's integration method (k+1)
        self.z_int     += sampleTime * ( z - self.z_d )
        self.theta_int += sampleTime * ssa( theta - theta_d )

        #######################################################################
        # Heading autopilot (SMC controller)
        #######################################################################
        
        wn_d = self.wn_d            # reference model natural frequency
        zeta_d = self.zeta_d        # reference model relative damping factor


        # Integral SMC with 3rd-order reference model
        [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
            integralSMC( 
                self.e_psi_int, 
                e_psi, e_r, 
                self.psi_d, 
                self.r_d, 
                self.a_d, 
                self.T_nomoto, 
                self.K_nomoto, 
                wn_d, 
                zeta_d, 
                self.K_d,
                self.K_sigma,
                self.lam,
                self.phi_b,
                psi_ref, 
                self.r_max, 
                sampleTime 
                )
            
        u_control = np.array([delta_r, -delta_r, delta_s, -delta_s, n], float)

        return u_control

    
