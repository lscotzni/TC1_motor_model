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

        R_expanded = csdl.expand(R, (num_nodes,1))
        L_d_expanded = csdl.expand(L_d, (num_nodes,1))
        L_q_expanded = csdl.expand(L_q, (num_nodes,1))
        phi_m_expanded = csdl.expand(phi_m, (num_nodes,1))

        a1 = self.register_output(
            'a1',
            (3*p*(omega*L_d_expanded-R_expanded))**2*(L_d_expanded-L_q_expanded)**2,
            # shape=(num_nodes,1)
        )

        a2 = self.register_output(
            'a2',
            18*p**2*(phi_m_expanded*omega**2*L_d_expanded*(2*L_d_expanded-L_q_expanded)* \
                (L_d_expanded-L_q_expanded)+phi_m_expanded*R_expanded**2*(L_d_expanded-L_q_expanded)),
            # shape=(num_nodes,1)
        )

        a3 = self.register_output(
            'a3',
            (3*phi_m_expanded*p*omega)**2*(6*L_d_expanded**2-6*L_d_expanded*L_q_expanded+L_q_expanded**2) + \
                9*phi_m_expanded**2*p**2 + R_expanded**2 - 9*(p*V_lim*(L_d_expanded-L_q_expanded))**2
        )

        a4 = self.register_output(
            'a4',
            18*phi_m_expanded**3*(p*omega)**2*(2*L_d_expanded-L_q_expanded) - \
                18*phi_m_expanded*(p*V_lim)**2*(L_d_expanded-L_q_expanded) - \
                12*omega*phi_m_expanded*p*R_expanded*T_em*(L_d_expanded-L_q_expanded)
        )

        a5 = self.register_output(
            'a5',
            4*L_d_expanded**2*T_em**2*omega**2 - 9*(phi_m_expanded*p*V_lim)**2 + 4*(R_expanded*T_em)**2 + \
                9*phi_m_expanded**4*(p*omega)**2 + 12*phi_m_expanded**2*p*R_expanded*T_em*omega
        )
        '''
        BRACKET FOR IMPLICIT FLUX WEAKENING MODEL
        '''
        I_d_asymp = self.register_output(
            'I_d_asymp',
            -phi_m_expanded/(L_d_expanded - L_q_expanded)
        ) # UPPER LIMIT OF I_d WHERE I_q ASYMPTOTES TO INFINITY

        # EVALUATE COEFFICIENTS FOR I_q FLUX WEAKENING METHOD FOR THE I_d BRACKETING
        a1_Iq = ((3*R_expanded*p*(L_d_expanded-L_q_expanded))**2 + \
            (3*L_d_expanded*L_q_expanded*omega*p - 3*L_q_expanded**2*omega*p)**2) / \
            (9*(p*(L_d_expanded-L_q_expanded))**2)
        a3_Iq = ((3*p*phi_m_expanded)**2*(R_expanded**2+omega**2*L_q_expanded**2)) / \
            (9*(p*(L_d_expanded-L_q_expanded))**2) - V_lim**2
        a4_Iq = -(12*p*phi_m_expanded*T_em*(omega**2*L_d_expanded*L_q_expanded-R_expanded**2)) / \
             (9*(p*(L_d_expanded-L_q_expanded))**2)

        p = a3_Iq / (2*a1_Iq)
        q = a4_Iq / (4*a1_Iq)
        I_q_hat = (-q/2 + (q**2/4+p**3/27)**(1/2))**(1/3) + (-q/2 - (q**2/4+p**3/27)**(1/2))**(1/3)
        T_em_lim = self.declare_variable('T_em_lim', shape=(num_nodes,1))
        I_d_hat = self.register_output(
            'I_d_hat',
            (2*T_em_lim/(3*p*I_q_hat) - phi_m_expanded)/(L_d_expanded-L_q_expanded)
        )

        '''
        IMPLICIT MODEL FOR FLUX WEAKENING
        '''
        model=Model()
        # BRACKET VARIABLES
        I_d_asymp = model.declare_variable('I_d_asymp')
        I_d_hat = model.declare_variable('I_d_hat')

        Id_fw = model.declare_variable('Id_fw', shape=(num_nodes,1)) # STATE

        a1 = model.declare_variable('a1', shape=(num_nodes,1))
        a2 = model.declare_variable('a2', shape=(num_nodes,1))
        a3 = model.declare_variable('a3', shape=(num_nodes,1))
        a4 = model.declare_variable('a4', shape=(num_nodes,1))
        a5 = model.declare_variable('a5', shape=(num_nodes,1))

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

        a1 = self.declare_variable('a1', shape=(num_nodes,1))
        a2 = self.declare_variable('a2', shape=(num_nodes,1))
        a3 = self.declare_variable('a3', shape=(num_nodes,1))
        a4 = self.declare_variable('a4', shape=(num_nodes,1))
        a5 = self.declare_variable('a5', shape=(num_nodes,1))

        Id_fw = solve_flux_weakening(a1, a2, a3, a4, a5)

''' 
TO-DO: 
    - ADD BRACKETS IN TERMS OF CSDL VARIABLES (comment out but have the form of the equation ready)



'''