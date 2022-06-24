import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

from TC1_motor_model.TC1_motor_sizing_model import TC1MotorSizingModel
from TC1_motor_model.TC1_motor_analysis_model import TC1MotorAnalysisModel

class TC1MotorModel(Model):
    '''
    INPUTS TO THIS MODEL:
        - tau_max (maximum torque)
        - rotor diameter
        - base speed of motor
    '''

    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('num_slots') # 36
        self.parameters.declare('op_voltage')
        self.parameters.declare('V_lim')
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H, B = f(H))
        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B, H = g(B))
        self.parameters.declare('num_nodes')

    def define(self):

        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']

        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,1))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,1))

        gear_ratio = 4

        omega = self.register_output('omega', omega_rotor * gear_ratio)
        load_torque = self.register_output('load_torque', load_torque_rotor/gear_ratio)

        self.add(
            TC1MotorSizingModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z
            ),
            'TC1_motor_sizing_model',
        )

        self.add(
            TC1MotorAnalysisModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                op_voltage=op_voltage,
                V_lim=V_lim,
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
                num_nodes=num_nodes,
            ),
            'TC1_motor_analysis_model',
        )


# NOTE:
#   - single motor sizing for entire aircraft (assuming all are identical)
#   - motor analysis model varies for the operating conditions of interest

if __name__ == '__main__':
    input_list = [6, 3, 36, 300, 800, 
        np.ones(11), np.ones(3), 4
    ]
    for i, val in enumerate(input_list):
        print(val)

    m = TC1MotorModel(
        pole_pairs=input_list[0],
        phases=input_list[1],
        num_slots=input_list[2],
        op_voltage=input_list[3],
        V_lim=input_list[4],
        fit_coeff_dep_H=input_list[5],
        fit_coeff_dep_B=input_list[6],
        num_nodes=input_list[7],
    )


    sim = Simulator(m)