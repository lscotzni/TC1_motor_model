import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class MTPAModel(Model):
    def initialize(self):
        self.parameters.declare('bracket_lower')
        self.parameters.declare('bracket_upper')
        self.parameters.declare('num_nodes')
        self.parameters.declare('op_voltage')

    def define(self):
        bracket_lb = self.parameters['bracket_lower']
        bracket_ub = self.parameters['bracket_upper']

        num_nodes = self.parameters['num_nodes']
        op_voltage = self.parameters['op_voltage']

        p = self.declare_variable('pole_pairs')
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_air = self.declare_variable('phi_air')
        T_em = self.declare_variable('T_em', shape=(num_nodes,))
        I_base = -phi_air/(L_d-L_q)

        T_em_star = self.register_output(
            'T_em_star',
            T_em/(1.5*p*I_base)
        ) # NON-DIM TORQUE

        model=Model()
        Iq_MTPA_star = model.declare_variable('Iq_MTPA_star', shape=(num_nodes,)) # NON-DIM Iq IN MTPA
        residual = model.register_output(
            'residual',
            Iq_MTPA_star**4 + T_em_star*Iq_MTPA_star - T_em_star**2,
        )

        solve_flux_weakening = self.create_implicit_operation(model)
        solve_flux_weakening.declare_state('Iq_MTPA_star', 
            residual='residual', 
            bracket=(bracket_lb, bracket_ub)
        )
        solve_flux_weakening.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=False,
        )
        solve_flux_weakening.linear_solver = ScipyKrylov()

        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_air = self.declare_variable('phi_air')
        T_em_star = self.declare_variable('T_em_star')
        Iq_MTPA_star = solve_flux_weakening(T_em_star)

        # --- ADD POST-PROCESSING OF THE OUTPUT IQ
        
        Iq_MTPA = self.register_output(
            'Iq_MTPA',
            Iq_MTPA_star * I_base
        ) # NEED TO DIMENSIONALIZE Iq_MTPA_star COMING FROM MTPA IMPLICIT SOLVER

''' 
TO-DO: 
    - ADD BRACKETS IN TERMS OF CSDL VARIABLES (comment out but have the form of the equation ready)



'''