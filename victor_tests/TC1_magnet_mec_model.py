import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class MagnetMECModel(Model):
    def initialize(self):
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H)
        self.parameters.declare('fit_form_dep_H') # FITTING EQUATION FORM (B = f(H))

        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B)
        self.parameters.declare('fit_form_dep_B') # FITTING EQUATION FORM (H = g(B))

        self.parameters.declare('num_nodes')
        self.parameters.declare('op_voltage')

    def fitting_dep_H(self, H):
        f = []
        order = 10
        for i in range(order + 1):
            f.append(self.fit_coeff_dep_H[i] * H**(order-i))
        return csdl.sum(*f)
    ''' FOR LOOP MIGHT BE A PROBLEM BECAUSE OBJECT H IS BEING APPENDED TO A NORMAL PYTHON LIST '''

    def fitting_dep_B(self, B):
        a = self.fit_coeff_dep_B[0]
        b = self.fit_coeff_dep_B[1]
        c = self.fit_coeff_dep_B[2]
        f = (a*csdl.exp(b*B + c) + 200) * B**1.4
        return f

    def define(self):
        self.fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        self.fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']
        op_voltage = self.parameters['op_voltage']

        MECmodel=Model()

        B_delta = MECmodel.declare_variable('B_delta') # AIR GAP FLUX DENSITY (STATE)

        ''' --- STATOR TOOTH CALCULATIONS --- '''
        t1 = MECmodel.declare_variable('tooth_pitch') # TOOTH PITCH
        b1 = MECmodel.declare_variable('tooth_width') # TOOTH WIDTH
        h_t1 = MECmodel.declare_variable('slot_height') # DEPTH OF SLOT (h_slot in MATLAB code)
        kfe = 0.95 # lamination coefficient
        
        B_t = B_delta * t1/b1/kfe # STATOR TOOTH FLUX DENSITY
        H_t = self.fitting_dep_B(B_t) # STATOR TOOTH MAGNETIC FIELD
        F_t = 2*H_t*h_t1 # MMF OF TOOTH

        ''' --- YOKE & ROTOR TOOTH CALCULATIONS --- '''
        alpha_i = MECmodel.declare_variable('alpha_i') # ELECTRICAL ANGLE PER SLOT
        tau = MECmodel.declare_variable('pole_pitch') # POLE PITCH
        l_ef = MECmodel.declare_variable('l_ef') # MAGNET LENGTH ALONG SHAFT (TYPICALLY STACK LENGTH)
        hj1 = MECmodel.declare_variable('height_yoke_stator') # YOKE HEIGHT IN STATOR
        ly = MECmodel.declare_variable('L_j1') # STATOR YOKE LENGTH OF MEC (CALLED L_j1 IN MATLAB & SIZING MODEL)

        phi_air = MECmodel.register_output(
            'phi_air',
            alpha_i*tau*l_ef*B_delta # AIR GAP FLUX; CALLED phi_air IN MATLAB CODE
        )
        B_y = phi_air / (2*hj1*l_ef) # YOKE FLUX DENSITY
        H_y = self.fitting_dep_B(B_y) # YOKE MAGNETIC FIELD
        MECmodel.register_output('H_y', H_y)
        F_y = 2*ly*H_y # YOKE MMF

        ''' --- AIR GAP MMF CALCULATIONS --- '''
        mu_0 = np.pi*4e-7
        sigma_air = MECmodel.declare_variable('air_gap_depth') # AIR GAP DEPTH
        K_theta = MECmodel.declare_variable('K_theta') # CARTER'S COEFF; CALLED K_theta IN MATLAB & SIZING MODEL

        F_delta = MECmodel.register_output(
            'F_delta',
            1.6*B_delta*(0.0001 + K_theta*sigma_air)/mu_0
        )
        
        ''' --- MMF SUM --- '''
        F_total = MECmodel.register_output(
            'F_total',
            F_t + F_y + F_delta
        )

        ''' --- MAGNETIC BRIDGE CALCULATIONS --- '''
        hm = MECmodel.declare_variable('magnet_thickness') # MAGNET THICKNESS
        H_f = F_total / hm
        B_f = self.fitting_dep_H(H_f)

        ''' --- LEAKAGE FLUX CALCULATIONS --- '''
        # NOTE: phi_air already calculated above
        A_f2 = MECmodel.declare_variable('A_f2') # CROSS SECTIONAL AREA OF MAGNET BRIDGE
        lambda_s = MECmodel.declare_variable('lambda_s', val=0.336e-6)
        # NOTE: lambda_s is not typically a constant so need to check later

        phi_f = MECmodel.register_output(
            'phi_f',
            B_f * A_f2
        )

        phi_s = MECmodel.register_output(
            'phi_s',
            F_total * lambda_s
        )
        
        phi_mag = phi_air + phi_f + phi_s
        phi_mag = MECmodel.register_output(
            'phi_mag',
            phi_mag
        )

        ''' --- MAGNET MMF CALCULATIONS --- '''
        bm = MECmodel.declare_variable('bm') # ARC LENGTH OF MAGNET
        lambda_leak = l_ef*bm*mu_0/0.0005
        F_m = F_total + phi_mag/lambda_leak # MAGNET MMF

        # RESIDUAL FLUX OF MAGNET (COMPUTED IN SIZING MODEL)
        phi_r = MECmodel.declare_variable('phi_r') 
        lambda_m = MECmodel.declare_variable('lambda_m')

        phi_air = phi_r - F_m*lambda_m

        residual = MECmodel.register_output(
            'residual',
            phi_air - phi_mag
        )

        ''' --- SETTING UP NONLINEAR SOLVER --- '''
        eps = 1e-6
        Br  = 1.2
        lower_bound = self.declare_variable('lower_bound', eps)
        upper_bound = self.declare_variable('upper_bound', Br)
        solve_MEC = self.create_implicit_operation(MECmodel)
        solve_MEC.declare_state('B_delta', residual='residual', bracket=(lower_bound, upper_bound))
        solve_MEC.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=True
        )
        solve_MEC.linear_solver = ScipyKrylov()

        ''' --- DECLARING VARIABLES AGAIN --- '''
        t1      = self.declare_variable('tooth_pitch', val=0.0244) # TOOTH PITCH
        b1      = self.declare_variable('tooth_width', val=0.0128) # TOOTH WIDTH
        h_t1      = self.declare_variable('slot_height', val=0.0180) # HEIGHT OF SLOT
        alpha_i   = self.declare_variable('alpha_i', val=0.8091) # ELECTRICAL ANGLE PER SLOT
        tau     = self.declare_variable('pole_pitch', val=.0732) # POLE PITCH
        l_ef    = self.declare_variable('l_ef', val=.2069) # MAGNET LENGTH ALONG SHAFT (TYPICALLY STACK LENGTH)
        hj1      = self.declare_variable('height_yoke_stator', val=0.0170) # YOKE HEIGHT IN STATOR
        ly      = self.declare_variable('L_j1', val=0.0435) # STATOR YOKE LENGTH OF MEC (CALLED L_j1 IN MATLAB & SIZING MODEL)
        sigma_air   = self.declare_variable('air_gap_depth', val=9.9469e-04) # AIR GAP DEPTH
        K_theta = self.declare_variable('K_theta', val=1.0835) # CARTER'S COEFF; CALLED K_theta IN MATLAB & SIZING MODEL
        hm     = self.declare_variable('magnet_thickness', val=0.0040) # MAGNET THICKNESS
        A_f2     = self.declare_variable('A_f2', val=8.2743e-04) # CROSS SECTIONAL AREA OF MAGNET BRIDGE
        lambda_s = self.declare_variable('lambda_s', val=0.336e-6)
        bm      = self.declare_variable('bm', val= 0.0563) # ARC LENGTH OF MAGNET
        phi_r   = self.declare_variable('phi_r', val=0.0130) 
        lambda_m = self.declare_variable('lambda_m', val=1.9245e-06)

        B_delta = solve_MEC(
            t1, b1, h_t1, alpha_i, tau, l_ef, hj1, ly, sigma_air, K_theta, 
            hm, A_f2, lambda_s, bm, phi_r, lambda_m
        )

        # B_delta, phi_air, phi_mag, H_y = solve_MEC(
        #     t1, b1, h_t1, alpha_i, tau, l_ef, hj1, ly, sigma_air, K_theta, 
        #     hm, A_f2, lambda_s, bm, phi_r, lambda_m, 
        #     expose=['phi_air', 'phi_mag', 'H_y']
        # )

        # --- MEC POST-PROCESSING ---
        hm = self.declare_variable('hm')
        mu_r = self.declare_variable('mu_r')
        Am_r = self.declare_variable('Am_r')
        # phi_air = self.declare_variable('phi_air')
        phi_f = self.declare_variable('phi_f')
        phi_s = self.declare_variable('phi_s')
        F_total = self.declare_variable('F_total')

        K_sigma_air = (phi_air+phi_f+phi_s)/phi_air

        lambda_theta = phi_air/F_total # MAIN MAGNETIC CONDUCTION
        lambda_theta_standard = (2*lambda_theta*hm) / (mu_r*mu_0*Am_r) # STANDARD VALUE OF lambda_theta
        lambda_n = self.register_output(
            'lambda_n',
            K_sigma_air*lambda_theta_standard
        )

        bm_0 =  lambda_n/(lambda_n + 1) # OPERATING POINT OF MAGNET
        lambda_leak_standard = self.register_output(
            'lambda_leak_standard', 
            (K_sigma_air - 1)*lambda_theta_standard
        )

    