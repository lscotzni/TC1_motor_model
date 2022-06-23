import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class MTPAModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('num_nodes')

    def define(self):
        p = self.parameters['pole_pairs']
        num_nodes = self.parameters['num_nodes']

        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_m = self.declare_variable('phi_m')
        T_em = self.declare_variable('T_em', shape=(num_nodes,1))
        I_base = -phi_m/(L_d-L_q)

        T_em_star = self.register_output(
            'T_em_star',
            T_em/(1.5*p*I_base)
        ) # NON-DIM TORQUE

        # DEPRESSED CUBIC METHOD
        p_c = T_em_star**2
        q_c = -T_em_star**2/8

        m = (-q_c/2 + (q_c**2/4+p_c**3/27)**(1/2))**(1/3) + (-q_c/2 - (q_c**2/4+p_c**3/27)**(1/2))**(1/3)
        Iq_MTPA_star = self.register_output(
            'Iq_MTPA_star',
            (-(2*m)**0.5 + (-(2*m-(T_em_star*2**0.5)/(m)**0.5))**0.5)/2
        )

        # --- ADD POST-PROCESSING OF THE OUTPUT Iq_MTPA_star
        Iq_MTPA = self.register_output(
            'Iq_MTPA',
            Iq_MTPA_star * I_base
        ) # NEED TO DIMENSIONALIZE Iq_MTPA_star COMING FROM MTPA IMPLICIT SOLVER

''' 
TO-DO: 
    - ADD BRACKETS IN TERMS OF CSDL VARIABLES (comment out but have the form of the equation ready)



'''