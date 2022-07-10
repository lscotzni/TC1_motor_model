import numpy as np
from csdl import Model, ScipyKrylov, NewtonSolver
from csdl_om import Simulator
import csdl

from TC1_motor_model.motor_submodels.TC1_magnet_mec_model import MagnetMECModel
from TC1_motor_model.motor_submodels.TC1_inductance_mec_model import InductanceModel
from TC1_motor_model.motor_submodels.TC1_flux_weakening_model import FluxWeakeningModel
from TC1_motor_model.motor_submodels.TC1_mtpa_model import MTPAModel
from TC1_motor_model.motor_submodels.TC1_post_processing_model import PostProcessingModel

'''
THIS MODEL CONTAINS THE ENTIRE MOTOR ANALYSIS MODEL.
THE GOAL OF THIS MODEL IS TO IMPLICITLY SOLVE FOR POWER AND EFFICIENCY
GIVEN A RESIDUAL CONSIDERING TORQUE AND EFFICIENCY.

FEOC = FOR EACH OPERATING CONDITION
INPUTS:
    - motor geometry (from sizing model)
    - RPM and load torque (FEOC)
    - voltage limit
    - voltage (FEOC); fed back in from the battery model

OUTPUTS:
    - power and efficiency (FEOC); fed into other models
    - EM torque (FEOC)
'''

class TC1MotorAnalysisModel(Model):
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

        self.motor_variable_names = [
            'outer_stator_radius', 'pole_pitch', 'tooth_pitch', 'air_gap_depth', 'l_ef',
            'rotor_radius', 'turns_per_phase', 'Acu',  'tooth_width', 'height_yoke_stator',
            'slot_bottom_width', 'slot_height', 'slot_width_inner', 'Tau_y', 'L_j1', 'Kdp1',
            'bm', 'Am_r', 'phi_r', 'lambda_m', 'alpha_i', 'Kf', 'K_phi', 'K_theta', 'A_f2',
        ]

    def define(self):
        # INPUT PARAMETERS
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        rated_current = self.parameters['rated_current']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']

        # DECLARE VARIABLES FROM SIZING & UPSTREAM MODELS
        D_i = self.declare_variable('D_i')
        # T_em_max = self.declare_variable('T_em_max')
        Rdc = self.declare_variable('Rdc')
        motor_variables = self.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs
        for i in range(motor_variables.shape[0]):
            self.register_output(self.motor_variable_names[i], motor_variables[i])

        omega_rotor = self.declare_variable('omega_rotor', shape=(num_nodes,))
        load_torque_rotor = self.declare_variable('load_torque_rotor', shape=(num_nodes,))

        # gearbox (very simple, not sure if this should be done differently)
        gear_ratio = 4
        omega = self.register_output('omega', omega_rotor * gear_ratio)
        load_torque = self.register_output('load_torque', load_torque_rotor/gear_ratio)

        # ========================= MAGNET MEC =========================
        self.add(
            MagnetMECModel(
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            ),
            'magnet_MEC_model',
        )

        # ========================= INDUCTANCE MEC =========================
        self.add(
            InductanceModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                rated_current=rated_current,
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            ),
            'inductance_MEC_model',
        )
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_air = self.declare_variable('phi_air')

        # ========================= IMPLICIT T_EM MODEL =========================
        model=Model()
        em_torque = model.declare_variable('em_torque', val=500, shape=(num_nodes,)) # STATE of implicit model

        load_torque = model.declare_variable('load_torque', shape=(num_nodes,))
        omega = model.declare_variable('omega', shape=(num_nodes,))
        
        Rdc = model.declare_variable('Rdc') # DC resistance
        implicit_motor_variables = model.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs
        for i in range(implicit_motor_variables.shape[0]):
            model.register_output(self.motor_variable_names[i], implicit_motor_variables[i])

        # FLUX WEAKENING MODEL
        model.add(
            FluxWeakeningModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            ),
            'flux_weakening_model',
        )

        # MTPA MODEL
        model.add(
            MTPAModel(
                pole_pairs=p,
                num_nodes=num_nodes
            ),
            'mtpa_model',
        )
        # POST-PROCESSING MODEL
        # model.add(
        #     PostProcessingModel(
        #         pole_pairs=p,
        #         phases=m,
        #         op_voltage=op_voltage,
        #         V_lim=V_lim,
        #         num_nodes=num_nodes
        #     ),
        #     'post_processing_model',
        # )
        Id_fw = model.declare_variable('Id_fw', shape=(num_nodes,))
        L_q = model.declare_variable('L_q')
        L_d = model.declare_variable('L_d')
        phi_air = model.declare_variable('phi_air')
        
        L_d_expanded = csdl.expand(L_d, (num_nodes,))
        L_q_expanded = csdl.expand(L_q, (num_nodes,))
        phi_m_expanded = csdl.expand(phi_air, (num_nodes,))

        Iq_fw = em_torque / (1.5*p * (phi_m_expanded + (L_d_expanded-L_q_expanded)*Id_fw)) # CHECK SIZE OF COMPUTATIONS HERE

        # smoothing
        Iq_MTPA = model.declare_variable('Iq_MTPA', shape=(num_nodes,)) # CHECK NAMING SCHEME FOR VARIABLE
        k = 1 # ASK SHUOFENG WHAT THIS IS
        I_q = (np.exp(k*(op_voltage - V_lim))*Iq_fw + np.exp(-1/(k*(op_voltage - V_lim)))*Iq_MTPA) / \
            (np.exp(k*(op_voltage - V_lim)) + np.exp(-1/(k*(op_voltage - V_lim))))
        I_d = (em_torque / (1.5*p*I_q) - phi_m_expanded) / (L_d_expanded-L_q_expanded) # CHECK SIZE OF COMPUTATIONS HERE
        current_amplitude = model.register_output(
            'current_amplitude',
            (I_q**2 + I_d**2)**0.5
        )
        ''' POWER LOSS CALCULATIONS '''
        # load power
        # eq of the form P0 = speed * torque
        P0 = load_torque * omega
        frequency = omega*p/60

        # copper loss
        Rdc_expanded = csdl.expand(Rdc, (num_nodes,))
        P_copper = m*Rdc_expanded*current_amplitude**2 # NEED TO CHECK IF THIS IS ELEMENT-WISE SQUARING

        # eddy_loss
        a = 0.00055 # lamination thickness in m
        sigma_c = 2e6 # bulk conductivity (2000000 S/m)
        l_ef = model.declare_variable('l_ef')
        D1 = model.declare_variable('outer_stator_radius')
        D_i = model.declare_variable('D_i')
        Acu = model.declare_variable('Acu')
        B_delta = model.declare_variable('B_delta')
        B_delta_expanded = csdl.expand(B_delta, (num_nodes,))
        
        K_e = (a*np.pi)**2 * sigma_c/6
        V_s = csdl.expand((np.pi*l_ef*(D1-D_i)**2)-36*l_ef*Acu, (num_nodes,)); # volume of stator
        K_c = 0.822;
        P_eddy = K_e*V_s*(B_delta_expanded*frequency)**2; # eddy loss

        # hysteresis loss
        K_h = 100
        n_h = 2
        P_h = K_h*V_s*frequency*B_delta_expanded**n_h

        # stress loss
        P_stress = 0.01*P0

        # windage & friction loss
        k_r = 4 # roughness coefficient
        fr = 3e-3 # friction coefficient
        rho_air = 1.225 # air density kg/m^3
        D2 = model.declare_variable('rotor_radius')
        l_ef_expanded = csdl.expand(l_ef, (num_nodes,))
        D2_expanded = csdl.expand(D2, (num_nodes,))
        P_wo = k_r*np.pi*fr*rho_air*(2*np.pi*frequency)**2*l_ef_expanded*D2_expanded**4

        # total losses
        P_loss = P_copper + P_eddy + P_h + P_stress + P_wo
        input_power = model.register_output('input_power', P0 + P_loss)
        efficiency = model.register_output('efficiency', P0/input_power)
        
        residual = model.register_output(
            'residual',
            load_torque - efficiency*em_torque
        )

        solve_motor_analysis = self.create_implicit_operation(model)
        solve_motor_analysis.declare_state(
            state='em_torque',
            residual='residual',
        )

        solve_motor_analysis.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=False,
        )
        solve_motor_analysis.linear_solver = ScipyKrylov()

        load_torque = self.declare_variable('load_torque', shape=(num_nodes,))
        omega = self.declare_variable('omega', shape=(num_nodes,))
        Rdc = self.declare_variable('Rdc')
        motor_variables = self.declare_variable('motor_variables', shape=(25,))
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        phi_air = self.declare_variable('phi_air')
        D_i = self.declare_variable('D_i')

        em_torque, efficiency, input_power, current_amplitude = solve_motor_analysis(
            load_torque, omega, Rdc, motor_variables, L_d, L_q, phi_air, D_i,     
            expose=['efficiency', 'input_power', 'current_amplitude']
        )
        # FROM HERE, THE OUTPUT POWER AND EFFICIENCY ARE PROPOGATED TO THE
        # BATTERY ANALYSIS MODELS

        




