import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model, GraphRepresentation
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

        D_i = self.declare_variable('D_i') # inner radius of stator
        L = self.declare_variable('L') # effective length of motor

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
        # outputs: resistance, mass, motor_variables
        self.declare_variable('T_em_max')

        # ---- MOTOR ANALYSIS MODEL ----
        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,))

        # variables that will feed into the motor analysis model
        self.declare_variable('Rdc') # DC resistance
        self.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs

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

        self.declare_variable('efficiency', shape=(num_nodes,))
        self.declare_variable('input_power', shape=(num_nodes,))
        self.declare_variable('current_amplitude', shape=(num_nodes,))


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
    op_voltage = 800
    V_lim = 1300
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

    rep = GraphRepresentation(m)
    sim = Simulator(rep)
    sim['D_i'] = 0.3723
    sim['L'] = 0.2755
    sim.run()
    # print('outer_stator_radius: ', sim['outer_stator_radius'])
    # print('pole_pitch: ', sim['pole_pitch'])
    # print('tooth_pitch: ', sim['tooth_pitch'])
    # print('air_gap_depth: ', sim['air_gap_depth'])
    # print('l_ef: ', sim['l_ef'])
    # print('rotor_radius: ', sim['rotor_radius'])
    # print('turns_per_phase: ', sim['turns_per_phase'])
    # print('Acu: ', sim['Acu'])
    # print('tooth_width: ', sim['tooth_width'])
    # print('height_yoke_stator: ', sim['height_yoke_stator'])
    # print('slot_bottom_width: ', sim['slot_bottom_width'])
    # print('slot_height: ', sim['slot_height'])
    # print('slot_width_inner: ', sim['slot_width_inner'])
    # print('Tau_y: ', sim['Tau_y'])
    # print('L_j1: ', sim['L_j1'])
    # print('Kdp1: ', sim['Kdp1'])
    # print('bm: ', sim['bm'])
    # print('Am_r: ', sim['Am_r'])
    # print('phi_r: ', sim['phi_r'])
    # print('lambda_m: ', sim['lambda_m'])
    # print('alpha_i: ', sim['alpha_i'])
    # print('Kf: ', sim['Kf'])
    # print('K_phi: ', sim['K_phi'])
    # print('K_theta: ', sim['K_theta'])
    # print('A_f2: ', sim['A_f2'])
    # print('Rdc: ', sim['Rdc'])
    # print('Rdc1: ', sim['Rdc1'])
    # print('Rac (incorrect): ', sim['Rac'])
    # print('motor mass: ', sim['motor_mass'])
    # print('----------')
    # print(sim['motor_variables'])
    print('input power:', sim['input_power'])
    sim.visualize_implementation()
