import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

class UpperLimitTorqueModel(Model):
    '''
    INPUTS TO THIS MODEL:
        - constant max torque (before base speed)
        - R, Ld, Lq, p,
        - omega for each operating condition

    OUTPUTS OF THIS MODEL:
        - base speed
        - max torque at each value of omega
    '''
    def initialize(self):
        # MOTOR DISCRETE PARAMETERS
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('num_slots') # 36
        self.parameters.declare('num_nodes')

    def define(self):
        # --- DEFINING INPUTS FROM INITIALIZE & BASIC PARAMETERS --- 
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        num_nodes=self.parameters['num_nodes']
        
        omega = self.declare_variable('omega', shape=(num_nodes,1))
        load_torque = self.declare_variable('load_torque', shape=(num_nodes,1))

        T_em_max = self.register_output(
            'T_em_max',
            omega*load_torque
        )