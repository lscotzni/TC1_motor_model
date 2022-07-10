import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

class MaxTorqueModel(Model):
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
        self.parameters.declare('fitting_order')

        self.fitting_coeff = {
            '0':[1],
            '1':[1, 1],
            '2':[1, 1, 1],
            '3':[1, 1, 1 ,1],
            '4':[1, 1, 1, 1, 1],
        }

    def fit_torque_to_mass(self, motor_mass):
        fitting_coeff = self.fitting_coeff.get(str(self.order))

        # torque_fitting = []
        # for i, val in enumerate(fitting_coeff):
        #     print(i, val)
        #     torque_fitting.append(val * motor_mass**i)
        # return csdl.sum(*torque_fitting)

        torque_fitting_array = self.create_output(
            'torque_fitting_array',
            shape=(self.order + 1,)
        )
        for i, val in enumerate(fitting_coeff):
            torque_fitting_array[i] = val * motor_mass**i
        return csdl.sum(torque_fitting_array)

    def define(self):
        self.order = self.parameters['fitting_order']
        # if self.parameters['fitting_order'] is None:
        #     self.order = self.parameters['fitting_order']
        # else:
        #     self.order = 2
        
        motor_mass = self.declare_variable('motor_mass')

        T_em_max = self.register_output(
            'T_em_max',
            self.fit_torque_to_mass(motor_mass)
        )