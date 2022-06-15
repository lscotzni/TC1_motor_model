import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class FluxWeakeningModel(Model):
    def initialize(self):
        self.parameters.declare('bracket_lower')
        self.parameters.declare('bracket_upper')
        self.parameters.declare('num_nodes')
        self.parameters.declare('op_voltage')
        self.parameters.declare('V_lim')

    def define(self):
        bracket_lb = self.parameters['bracket_lower']
        bracket_ub = self.parameters['bracket_upper']

        num_nodes = self.parameters['num_nodes']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']

        R = self.declare_variable('Rdc')
        p = self.declare_variable('pole_pairs')
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_m = self.declare_variable('phi_m')
        omega = self.declare_variable('omega', shape=(num_nodes,1))
        T_em = self.declare_variable('T_em', shape=(num_nodes,1))

        a1 = self.declare_variable(
            'a1',
            3*R*p*(L_d-L_q)**2, ''' NEEDS FIXING '''
        )

        a2 = self.declare_variable(
            'a2',
            18*p**2, ''' NEEDS FIXING '''
        )

        a3 = self.declare_variable(
            'a3',
            18*omega*p**2*phi_m*(L_d-L_q)*(3*L_d-L_q) + 9*(phi_m*p*R)**2 - \
                12*omega*p*R*T_em*(L_d**2-L_d*L_q) - 9*(p*V_lim*(L_d-L_q))**2
        )

        a4 = self.declare_variable(
            'a4',
            18*(phi_m*p)**2*omega*(3*L_d-2*L_q) - 18*(p*V_lim)**2*(L_d-L_q) - \
                12*p*R*T_em*(omega*L_d*phi_m - (L_d-L_q))
        )

        a5 = self.declare_variable(
            'a5',
            4*L_d**2*T_em**2*omega**2 + 18*phi_m**3*p**2*omega + 12*phi_m*p*R*T_em \
                - 9*(phi_m*p**V_lim)**2
        )

        model=Model()
        Id_fw = model.declare_variable('Id_fw')

        a1 = model.declare_variable('a1')
        a2 = model.declare_variable('a2')
        a3 = model.declare_variable('a3')
        a4 = model.declare_variable('a4')
        a5 = model.declare_variable('a5')

        residual = model.register_output(
            'residual',
            a1*Id_fw**4 + a2*Id_fw**3 + a3*Id_fw**2 + a4*Id_fw + a5
        )

        solve_flux_weakening = self.create_implicit_operation(model)
        solve_flux_weakening.declare_state('Id_fw', 
            residual='residual', 
            bracket=(bracket_lb, bracket_ub)
        )
        solve_flux_weakening.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=False,
        )
        solve_flux_weakening.linear_solver = ScipyKrylov()

        a1 = self.declare_variable('a1')
        a2 = self.declare_variable('a2')
        a3 = self.declare_variable('a3')
        a4 = self.declare_variable('a4')
        a5 = self.declare_variable('a5')

        Id_fw = solve_flux_weakening(a1, a2, a3, a4, a5)

''' 
TO-DO: 
    - ADD BRACKETS IN TERMS OF CSDL VARIABLES (comment out but have the form of the equation ready)



'''