import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

from TC1_motor_model.TC1_motor_sizing_model_new import TC1MotorSizingModel
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
        self.parameters.declare('rated_current')
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H, B = f(H))
        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B, H = g(B))
        self.parameters.declare('num_nodes')

    def define(self):

        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        rated_current = self.parameters['rated_current']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']

        # ---- MOTOR SIZING MODEL ----
        self.add(
            TC1MotorSizingModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                rated_current=rated_current
            ),
            'TC1_motor_sizing_model',
        )

        # ---- MOTOR ANALYSIS MODEL ----
        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,1))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,1))

        # gearbox (very simple, not sure if this should be done differently)
        gear_ratio = 4
        omega = self.register_output('omega', omega_rotor * gear_ratio)
        load_torque = self.register_output('load_torque', load_torque_rotor/gear_ratio)

        # variables that will feed into the motor analysis model
        self.declare_variable('Rdc') # DC resistance
        self.declare_variable('motor_mass') # motor mass
        self.declare_variable('motor_variables', shape=(23,1)) # array of motor sizing outputs

        self.add(
            TC1MotorAnalysisModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                op_voltage=op_voltage,
                V_lim=V_lim,
                rated_current=rated_current,
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
                num_nodes=num_nodes,
            ),
            'TC1_motor_analysis_model',
        )

        self.declare_variable('efficiency')
        self.declare_variable('input_power')


# NOTE:
#   - single motor sizing for entire aircraft (assuming all are identical)
#   - motor analysis model varies for the operating conditions of interest

if __name__ == '__main__':
    # PERMEABILITY FITTING IMPORT + GENERATION
    from TC1_motor_model.permeability.mu_fitting import permeability_fitting
    file_name = 'Magnetic_alloy_silicon_core_iron_C.tab'
    mu_fitting = permeability_fitting(file_name=file_name)

    fit_coeff_dep_H = mu_fitting[0]
    fit_coeff_dep_B = mu_fitting[1]

    p = 6
    m = 3
    Z = 36
    op_voltage = 300
    V_lim = 800
    rated_current = 123
    num_nodes = 4 # dummy input

    m = TC1MotorModel(
        pole_pairs=p,
        phases=m,
        num_slots=Z,
        op_voltage=op_voltage,
        V_lim=V_lim,
        rated_current=rated_current,
        fit_coeff_dep_H=fit_coeff_dep_H,
        fit_coeff_dep_B=fit_coeff_dep_B,
        num_nodes=num_nodes,
    )

    sim = Simulator(m)

    sim.visualize_implementation()