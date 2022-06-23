import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class FluxWeakeningModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']
        
        R = self.declare_variable('Rdc')
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_m = self.declare_variable('phi_m')
        omega = self.declare_variable('omega', shape=(num_nodes,1))
        T_em = self.declare_variable('T_em', shape=(num_nodes,1))

        a1 = self.declare_variable(
            'a1',
            (3*p*(omega*L_d-R))**2*(L_d-L_q)**2,
            shape=(num_nodes,1)
        )

        a2 = self.declare_variable(
            'a2',
            18*p**2*(phi_m*omega**2*L_d*(2*L_d-L_q)*(L_d-L_q)+phi_m*R**2*(L_d-L_q)),
            shape=(num_nodes,1)
        )

        a3 = self.declare_variable(
            'a3',
            (3*phi_m*p*omega)**2*(6*L_d**2-6*L_d*L_q+L_q**2) + 9*phi_m**2*p**2 + \
                R**2 - 9*(p*V_lim*(L_d-L_q))**2,
            shape=(num_nodes,1)
        )

        a4 = self.declare_variable(
            'a4',
            18*phi_m**3*(p*omega)**2*(2*L_d-L_q) - 18*phi_m*(p*V_lim)**2*(L_d-L_q) - \
                12*omega*phi_m*p*R*T_em*(L_d-L_q),
            shape=(num_nodes,1)
        )

        a5 = self.declare_variable(
            'a5',
            4*L_d**2*T_em**2*omega**2 - 9*(phi_m*p*V_lim)**2 + 4*(R*T_em)**2 + \
                9*phi_m**4*(p*omega)**2 + 12*phi_m**2*p*R*T_em*omega,
            shape=(num_nodes,1)
        )
        '''
        BRACKET FOR IMPLICIT FLUX WEAKENING MODEL
        '''
        I_d_asymp = self.register_output(
            'I_d_asymp',
            -phi_m/(L_d - L_q)
        ) # UPPER LIMIT OF I_d WHERE I_q ASYMPTOTES TO INFINITY

        # EVALUATE COEFFICIENTS FOR I_q FLUX WEAKENING METHOD FOR THE I_d BRACKETING
        a1_Iq = ((3*R*p*(L_d-L_q))**2 + (3*L_d*L_q*omega*p - 3*L_q**2*omega*p)**2) / \
            (9*(p*(L_d-L_q))**2)
        a3_Iq = ((3*p*phi_m)**2*(R**2+omega**2*L_q**2))/(9*(p*(L_d-L_q))**2) - V_lim**2
        a4_Iq = -(12*p*phi_m*T_em*(omega**2*L_d*L_q-R**2)) / (9*(p*(L_d-L_q))**2)

        p = a3_Iq / (2*a1_Iq)
        q = a4_Iq / (4*a1_Iq)
        I_q_hat = (-q/2 + (q**2/4+p**3/27)**(1/2))**(1/3) + (-q/2 - (q**2/4+p**3/27)**(1/2))**(1/3)
        T_em_lim = self.declare_variable('T_em_lim')
        I_d_hat = self.register_output(
            'I_d_hat',
            (2*T_em_lim/(3*p*I_q_hat) - phi_m)/(L_d-L_q)
        )

        '''
        IMPLICIT MODEL FOR FLUX WEAKENING
        '''
        model=Model()
        # BRACKET VARIABLES
        I_d_asymp = model.declare_variable('I_d_asymp')
        I_d_hat = model.declare_variable('I_d_hat')

        Id_fw = model.declare_variable('Id_fw') # STATE

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
            bracket=(I_d_hat, I_d_asymp)
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