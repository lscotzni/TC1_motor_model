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
        self.parameters.declare('num_nodes')
        self.parameters.declare('op_voltage')

    def define(self):

        num_nodes = self.parameters['num_nodes']
        op_voltage = self.parameters['op_voltage']

        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,1))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,1))

        gear_ratio = 4

        omega = self.register_output('omega', omega_rotor * gear_ratio)
        load_torque = self.register_output('load_torque', load_torque_rotor/gear_ratio)

        self.add(
            TC1MotorSizingModel(),
            'TC1_motor_sizing_model',
        )

        self.add(
            TC1MotorAnalysisModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage
            ),
            'TC1_motor_analysis_model',
        )


# NOTE:
#   - single motor sizing for entire aircraft (assuming all are identical)
#   - motor analysis model varies for the operating conditions of interest