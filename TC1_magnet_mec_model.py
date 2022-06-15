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

        B_delta = MECmodel.declare_variable('B_delta') # AIR GAP FLUX DENSITY

        ''' --- STATOR TOOTH CALCULATIONS --- '''
        t = MECmodel.declare_variable('t') # TOOTH PITCH
        bt = MECmodel.declare_variable('bt') # TOOTH WIDTH
        ht = MECmodel.declare_variable('ht') # HEIGHT OF SLOT
        
        B_t = B_delta * t/bt # STATOR TOOTH FLUX DENSITY
        H_t = self.fitting_dep_B(B_t) # STATOR TOOTH MAGNETIC FIELD
        F_t = 2*H_t*ht # MMF OF TOOTH

        ''' --- YOKE & ROTOR TOOTH CALCULATIONS --- '''
        alpha = MECmodel.declare_variable('alpha') # ELECTRICAL ANGLE PER SLOT
        tau = MECmodel.declare_variable('pole_pitch') # POLE PITCH
        l_ef = MECmodel.declare_variable('l_ef') # MAGNET LENGTH ALONG SHAFT (TYPICALLY STACK LENGTH)
        hy = MECmodel.declare_variable('hy') # YOKE HEIGHT IN STATOR
        ly = MECmodel.declare_variable('L_j1') # STATOR YOKE LENGTH OF MEC (CALLED L_j1 IN MATLAB & SIZING MODEL)

        phi_delta = alpha*tau*l_ef*B_delta # AIR GAP FLUX
        B_y = phi_delta / (2*hy*l_ef) # YOKE FLUX DENSITY
        H_y = self.fitting_dep_B(B_y) # YOKE MAGNETIC FIELD
        F_y = 2*ly*H_y # YOKE MMF

        ''' --- AIR GAP MMF CALCULATIONS --- '''
        mu_0 = np.pi*4e-7
        delta = MECmodel.declare_variable('delta') # AIR GAP DEPTH
        k_delta = MECmodel.declare_variable('K_theta') # CARTER'S COEFF; CALLED K_theta IN MATLAB & SIZING MODEL

        F_delta = self.register_output(
            'F_delta',
            1.6*delta*k_delta*B_delta/mu_0
        )
        
        ''' --- MMF SUM --- '''
        F_sum = self.register_output(
            'F_sum',
            F_t + F_y + F_delta
        )

        ''' --- MAGNETIC BRIDGE CALCULATIONS --- '''
        l_f = MECmodel.declare_variable('l_f') # MAGNET THICKNESS
        H_f = F_sum / l_f
        B_f = self.fitting_dep_H(H_f)

        ''' --- LEAKAGE FLUX CALCULATIONS --- '''
        # NOTE: phi_delta already calculated above
        A_f = MECmodel.declare_variable('A_f2') # CROSS SECTIONAL AREA OF MAGNET BRIDGE
        lambda_s = MECmodel.declare_variable('lambda_s', val=0.336e-6)
        # NOTE: lambda_s is not typically a constant so need to check later

        phi_f = B_f * A_f
        phi_s = F_sum * lambda_s
        phi_sum = phi_delta + phi_f + phi_s
        phi_sum = self.register_output(
            'phi_sum',
            phi_sum
        )

        ''' --- MAGNET MMF CALCULATIONS --- '''
        bm = MECmodel.declare_variable('bm') # ARC LENGTH OF MAGNET
        sigma = 0.0005 # AVG TOLERANCE BETWEEN MAGNET AND LAMINATION STEEL ASSUMED AS CONSTANT
        lambda_delta = l_ef*bm*mu_0/sigma
        F_m = F_sum + phi_sum/lambda_delta # MAGNET MMF

        # RESIDUAL FLUX OF MAGNET (COMPUTED IN SIZING MODEL)
        phi_r = MECmodel.declare_variable('phi_r') 
        lambda_m = MECmodel.declare_variable('lambda_m')

        phi_m = phi_r - F_m*lambda_m

        residual = MECmodel.register_output(
            'residual',
            phi_m - phi_sum
        )

        ''' --- SETTING UP NONLINEAR SOLVER --- '''
        eps = 1e-5
        Br  = 1.2
        solve_MEC = self.create_implicit_operation(MECmodel)
        solve_MEC.declare_state('B_delta', residual='residual', bracket=(eps, Br - eps))
        solve_MEC.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=True
        )
        solve_MEC.linear_solver = ScipyKrylov()

        ''' --- DECLARING VARIABLES AGAIN --- '''
        t       = self.declare_variable('t') # TOOTH PITCH
        bt      = self.declare_variable('bt') # TOOTH WIDTH
        ht      = self.declare_variable('ht') # HEIGHT OF SLOT
        alpha   = self.declare_variable('alpha') # ELECTRICAL ANGLE PER SLOT
        tau     = self.declare_variable('pole_pitch') # POLE PITCH
        l_ef    = self.declare_variable('l_ef') # MAGNET LENGTH ALONG SHAFT (TYPICALLY STACK LENGTH)
        hy      = self.declare_variable('hy') # YOKE HEIGHT IN STATOR
        ly      = self.declare_variable('L_j1') # STATOR YOKE LENGTH OF MEC (CALLED L_j1 IN MATLAB & SIZING MODEL)
        delta   = self.declare_variable('delta') # AIR GAP DEPTH
        k_delta = self.declare_variable('K_theta') # CARTER'S COEFF; CALLED K_theta IN MATLAB & SIZING MODEL
        l_f     = self.declare_variable('l_f') # MAGNET THICKNESS
        A_f     = self.declare_variable('A_f2') # CROSS SECTIONAL AREA OF MAGNET BRIDGE
        lambda_s = self.declare_variable('lambda_s', val=0.336e-6)
        bm      = self.declare_variable('bm') # ARC LENGTH OF MAGNET
        phi_r   = self.declare_variable('phi_r') 
        lambda_m = self.declare_variable('lambda_m')

        B_delta = solve_MEC(
            t, bt, ht, alpha, tau, l_ef, hy, ly, delta, k_delta, 
            l_f, A_f, lambda_s, bm, phi_r, lambda_m
        )

        # --- MEC POST-PROCESSING ---
        hm = self.declare_variable('hm')
        mu_r = self.declare_variable('mu_r')
        Am_r = self.declare_variable('Am_r')
        K_sigma_air = self.declare_variable('K_sigma_air')

        lambda_theta = phi_sum/F_sum # MAIN MAGNETIC CONDUCTION
        lambda_theta_standard = (2*lambda_theta*hm) / (mu_r*mu_0*Am_r) # STANDARD VALUE OF lambda_theta
        lambda_n = self.register_output(
            'lambda_n',
            K_sigma_air*lambda_theta_standard
        )

        bm_0 =  lambda_n/(lambda_n + 1) # OPERATING POINT OF MAGNET
        lambda_leak_standard = (K_sigma_air - 1)*lambda_theta_standard
        lambda_leak_standard = self.register_output('lambda_leak_standard', lambda_leak_standard)



if __name__ == '__main__':
    from mu_fitting import permeability_fitting
    file_name = 'Magnetic alloy, silicon core iron C.tab'
    order=10

    mu_fitting = permeability_fitting(
        file_name=file_name,
        test=True,
    )

    fit_coeff_dep_H = mu_fitting[0]
    fit_coeff_dep_B = mu_fitting[1]

    m = MagnetMECModel(
        fit_coeff_dep_H=fit_coeff_dep_H,
        fit_coeff_dep_B=fit_coeff_dep_B
    )

    sim = Simulator(m)
    print(sim['B_delta'])
    sim.run()
    print(sim['B_delta'])
    