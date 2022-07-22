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
        self.declare_variable('Rdc') # DC resistance
        self.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs

        # ---- MOTOR ANALYSIS MODEL ----
        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,))

        

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



        # self.declare_variable('efficiency', shape=(num_nodes,))
        # self.declare_variable('input_power', shape=(num_nodes,))
        # self.declare_variable('current_amplitude', shape=(num_nodes,))


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
    op_voltage = 500
    V_lim = 1300
    rated_current = 123
    num_nodes = 4 # dummy input

    # D_i = 0.3723
    # l_ef = 0.2755

    D_i = 0.182
    l_ef = 0.086


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
    sim['D_i'] = D_i
    sim['L'] = l_ef
    sim['omega_rotor'] = 1000/4
    sim['load_torque_rotor'] = 2000*4
    sim['T_em'] = 1500
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
    print('Rdc: ', sim['Rdc'])
    # print('Rdc1: ', sim['Rdc1'])
    # print('Rac (incorrect): ', sim['Rac'])
    print('motor mass: ', sim['motor_mass'])
    print('T_em_max: ', sim['T_em_max'])
    print('Ld: ', sim['L_d'])
    print('Lq: ', sim['L_q'])
    print('phi_air: ', sim['phi_air'])
    print('PsiF: ', sim['PsiF'])

    print(' ---- OUTPUTS FROM TORQUE LIMIT MODEL ---- ')
    print('torque: (found implicitly)', sim['T_lim'])
    print(sim['max_cubic_root'])
    print(sim['upper_quartic_bracket'])
    print(sim['A_quartic'])
    print(sim['B_quartic'])
    print(sim['C_quartic'])
    print(sim['D_quartic'])
    print(sim['E_quartic'])
    print('------')
    # print(sim['test_out'])
    print(sim['cubic_roots'])

    print(' ---- OUTPUTS FROM FLUX WEAKENING ---- ')
    print('Iq bracket coeff a: ', sim['a_bracket'])
    print('Iq bracket coeff c: ', sim['c_bracket'])
    print('Iq bracket coeff d: ', sim['d_bracket'])
    print('Iq bracket coeff e: ', sim['e_bracket'])
    print('Iq FW bracket: ', sim['I_q_fw_bracket'])
    print('Id lower bracket: ', sim['Id_fw_bracket'])
    print('Id upper bracket: ', sim['I_d_asymp'])
    print('Flux Weakening Iq: ', sim['Iq_fw'])
    print('Flux Weakening Id: ', sim['Id_fw'])

    print(' ---- OUTPUTS FROM MTPA ---- ')
    print('MTPA non-dim Iq: ', sim['Iq_MTPA_star'])
    print('MTPA Iq: ', sim['Iq_MTPA'])

    print(' ---- OUTPUTS FROM POST PROCESSING ---- ')
    print('Current Amplitude: ', sim['current_amplitude'])
    print('Voltage Amplitude: ', sim['voltage_amplitude'])
    print('Output Power: ', sim['output_power'])
    print('Input Power: ', sim['input_power'])
    print('Efficiency: ', sim['efficiency'])
    


    # print('----------')
    # print(sim['motor_variables'])
    # print('input power:', sim['input_power'])
    # sim.visualize_implementation()
