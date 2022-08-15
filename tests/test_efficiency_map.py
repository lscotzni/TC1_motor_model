import numpy as np 
import csdl
from python_csdl_backend import Simulator
import matplotlib.pyplot as plt

from TC1_motor_model.motor_submodels.TC1_magnet_mec_model import MagnetMECModel
from TC1_motor_model.motor_submodels.TC1_inductance_mec_model import InductanceModel
from TC1_motor_model.motor_submodels.TC1_torque_limit_model import TorqueLimitModel
from TC1_motor_model.motor_submodels.TC1_flux_weakening_model import FluxWeakeningBracketModel
from TC1_motor_model.motor_submodels.TC1_efficiency_map_model import EfficiencyMapModel
from TC1_motor_model.TC1_motor_sizing_model import TC1MotorSizingModel

class ExtractUpperLimitTorqueModel(csdl.Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('num_slots') # 36
        self.parameters.declare('op_voltage')
        self.parameters.declare('V_lim')
        self.parameters.declare('rated_current')
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H, B = f(H))
        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B, H = g(B))
        self.parameters.declare('num_active_nodes')

        self.motor_variable_names = [
            'outer_stator_radius', 'pole_pitch', 'tooth_pitch', 'air_gap_depth', 'l_ef',
            'rotor_radius', 'turns_per_phase', 'Acu',  'tooth_width', 'height_yoke_stator',
            'slot_bottom_width', 'slot_height', 'slot_width_inner', 'Tau_y', 'L_j1', 'Kdp1',
            'bm', 'Am_r', 'phi_r', 'lambda_m', 'alpha_i', 'Kf', 'K_phi', 'K_theta', 'A_f2',
        ]

    def define(self):
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        rated_current = self.parameters['rated_current']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_active_nodes = self.parameters['num_active_nodes']

        D_i = self.declare_variable('D_i') # inner radius of stator
        L = self.declare_variable('L') # effective length of motor

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
        Rdc = self.declare_variable('Rdc') # DC resistance
        motor_variables = self.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs
        for i in range(motor_variables.shape[0]):
            self.register_output(self.motor_variable_names[i], motor_variables[i])

        self.declare_variable('omega', shape=num_active_nodes)

        self.add(
            MagnetMECModel(
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            ),
            'magnet_MEC_model',
        )

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
        W_1 = self.declare_variable('turns_per_phase')
        PsiF = self.register_output('PsiF', W_1 * phi_air)

        R_expanded = self.register_output('R_expanded', csdl.expand(Rdc, (num_active_nodes,)))
        L_d_expanded = self.register_output('L_d_expanded', csdl.expand(L_d, (num_active_nodes,)))
        L_q_expanded = self.register_output('L_q_expanded', csdl.expand(L_q, (num_active_nodes,)))
        PsiF_expanded = self.register_output('PsiF_expanded', csdl.expand(PsiF, (num_active_nodes,)))

        self.add(
                TorqueLimitModel(
                    pole_pairs=p,
                    V_lim=V_lim,
                    num_nodes=num_active_nodes
                ),
                'torque_limit_model'
        )
        # OUTPUT OF torque_limit_model GIVES THE UPPER LIMIT CURVE BASED ON FLUX WEAKENING

        T_em_max = self.declare_variable('T_em_max')
        T_lim = self.declare_variable('T_lim', shape=(num_active_nodes,))

        T_upper_lim_curve = self.register_output(
                'T_upper_lim_curve',
                # -csdl.log(csdl.exp(-T_lim) + csdl.exp(-csdl.expand(T_em_max, (num_active_nodes,))))
                csdl.min(csdl.reshape(T_lim, new_shape=(num_active_nodes, )),
                            csdl.expand(T_em_max, (num_active_nodes, )))
            )

class EfficiencyMapAnalysisModel(csdl.Model):
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
        self.parameters.declare('num_active_nodes')

        self.motor_variable_names = [
            'outer_stator_radius', 'pole_pitch', 'tooth_pitch', 'air_gap_depth', 'l_ef',
            'rotor_radius', 'turns_per_phase', 'Acu',  'tooth_width', 'height_yoke_stator',
            'slot_bottom_width', 'slot_height', 'slot_width_inner', 'Tau_y', 'L_j1', 'Kdp1',
            'bm', 'Am_r', 'phi_r', 'lambda_m', 'alpha_i', 'Kf', 'K_phi', 'K_theta', 'A_f2',
        ]

    def define(self):

        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        rated_current = self.parameters['rated_current']
        num_active_nodes = self.parameters['num_active_nodes']

        D_i = self.declare_variable('D_i') # inner radius of stator

        Rdc = self.declare_variable('Rdc')
        L_d = self.declare_variable('L_d')
        L_q = self.declare_variable('L_q')
        PsiF = self.declare_variable('PsiF')

        omega = self.declare_variable('omega', shape=(num_active_nodes,))
        T_em = self.declare_variable('T_em', shape=(num_active_nodes,))

        T_lim = self.declare_variable('T_lim', shape=(num_active_nodes,))
        T_lower_lim = self.declare_variable('T_lower_lim', val=0., shape=(num_active_nodes,))

        R_expanded = self.register_output('R_expanded', csdl.expand(Rdc, (num_active_nodes,)))
        L_d_expanded = self.register_output('L_d_expanded', csdl.expand(L_d, (num_active_nodes,)))
        L_q_expanded = self.register_output('L_q_expanded', csdl.expand(L_q, (num_active_nodes,)))
        PsiF_expanded = self.register_output('PsiF_expanded', csdl.expand(PsiF, (num_active_nodes,)))

        D = (3*p*(L_d_expanded-L_q_expanded))

        # FLUX WEAKENING BRACKETING IMPLICIT MODEL (TO FIND BRACKET LIMIT)
        a_bracket = self.register_output(
            'a_bracket',
            D**2 * ((omega*L_q_expanded)**2 + R_expanded**2)
        )
        c_bracket = self.register_output(
            'c_bracket',
            (3*p*PsiF_expanded)**2 * (R_expanded**2 + (omega*L_q_expanded)**2) + \
            12*p*omega*R_expanded*T_lim*(L_d_expanded-L_q_expanded)**2 - (V_lim*D)**2
        )
        d_bracket = self.register_output(
            'd_bracket',
            -12*p*PsiF_expanded*T_lim*(R_expanded**2 + omega**2*L_d_expanded*L_q_expanded)
        )
        e_bracket = self.register_output(
            'e_bracket',
            4*T_lim**2*(R_expanded**2 + (omega*L_d_expanded)**2)
        )

        self.add(
            FluxWeakeningBracketModel(
                pole_pairs=p,
                num_nodes=num_active_nodes
            ),
            'flux_weakening_bracket_method'
        )

        Iq_fw_bracket = self.declare_variable('Iq_fw_bracket', shape=(num_active_nodes, ))
        Id_fw_bracket = self.declare_variable('Id_fw_bracket', shape=(num_active_nodes, ))

        self.add(
            EfficiencyMapModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_active_nodes,
                rated_current=rated_current,
                phases=m,
                motor_variable_names=self.motor_variable_names
            ),
            'efficiency_map_model'
        )


if __name__ == '__main__':
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

    D_i = 0.182
    L = 0.086

    omega_step = 60 # TO ADJUST NUMBER OF STEPS BETWEEN LOWER (NONZERO) AND UPPER OMEGA
    omega_range = np.linspace(100, 20000, omega_step)
    num_active_nodes = len(omega_range)

    ''' INITIAL MODEL EVALUATION TO DETERMINE THE UPPER LIMIT EM TORQUE CURVE '''
    torque_lim_model = ExtractUpperLimitTorqueModel(
        pole_pairs=p,
        phases=m,
        num_slots=Z,
        op_voltage=op_voltage,
        V_lim=V_lim,
        rated_current=rated_current,
        fit_coeff_dep_H=fit_coeff_dep_H,
        fit_coeff_dep_B=fit_coeff_dep_B,
        num_active_nodes=num_active_nodes,
    )
    rep = csdl.GraphRepresentation(torque_lim_model)
    sim = Simulator(rep)
    sim['D_i'] = D_i
    sim['L'] = L
    sim['omega'] = omega_range
    sim.run()
    upper_torque_curve = sim['T_upper_lim_curve']
    torque_voltage_limit = sim['T_lim']
    R = sim['Rdc']
    L_d = sim['L_d']
    L_q = sim['L_q']
    PsiF = sim['PsiF']
    B_delta = sim['B_delta']
    motor_variables = sim['motor_variables']
    
    # fig = plt.figure(1)
    # plt.plot(omega_range, upper_torque_curve, 'k', linewidth=5)
    # plt.xlabel('RPM')
    # plt.ylabel('EM Torque')

    torque_grid_step = 60 # TO ADJUST NUMBER OF STEPS BETWEEN LOWER (NONZERO) AND UPPER LIMIT TORQUE
    omega_grid = np.resize(omega_range, (torque_grid_step, len(omega_range)))
    torque_grid = np.linspace(10, upper_torque_curve, torque_grid_step)
    torque_voltage_limit_grid = np.resize(torque_voltage_limit, (torque_grid_step, len(omega_range)))

    omega_grid_vector = np.reshape(omega_grid, (torque_grid_step * len(omega_range), ))
    torque_grid_vector = np.reshape(torque_grid, (torque_grid_step * len(omega_range), ))
    torque_voltage_limit_grid_vector = np.reshape(torque_voltage_limit_grid, (torque_grid_step * len(omega_range), ))

    efficiency_map_analysis_model = EfficiencyMapAnalysisModel(
        pole_pairs=p,
        phases=m,
        num_slots=Z,
        op_voltage=op_voltage,
        V_lim=V_lim,
        rated_current=rated_current,
        num_active_nodes=len(omega_grid_vector),
    )

    rep_eff = csdl.GraphRepresentation(efficiency_map_analysis_model)
    sim_eff_map = Simulator(rep_eff)
    sim_eff_map['omega'] = omega_grid_vector
    sim_eff_map['T_em'] = torque_grid_vector
    sim_eff_map['Rdc'] = R
    sim_eff_map['L_d'] = L_d
    sim_eff_map['L_q'] = L_q 
    sim_eff_map['PsiF'] = PsiF 
    sim_eff_map['T_lim'] = torque_voltage_limit_grid_vector
    sim_eff_map['D_i'] = D_i
    sim_eff_map['B_delta'] = B_delta
    sim_eff_map['motor_variables'] = motor_variables
    sim_eff_map.run()

    efficiency_active = sim_eff_map['efficiency_active']

    efficiency_grid = np.zeros((torque_grid_step + 1,len(omega_range) + 1))
    efficiency_grid[1:, 1:] = np.reshape(efficiency_active, (torque_grid_step, len(omega_range)))
    omega_grid_plot = np.zeros_like(efficiency_grid)
    omega_grid_plot[1:, 1:] = np.reshape(omega_grid, (torque_grid_step, len(omega_range)))
    omega_grid_plot[0, 1:] = omega_range
    torque_grid_plot = np.zeros_like(efficiency_grid)
    torque_grid_plot[1:, 1:] = np.reshape(torque_grid, (torque_grid_step, len(omega_range)))
    torque_grid_plot[1:, 0] = torque_grid_plot[1:,1]

    levels_f = np.linspace(0,1,11)
    # levels_f = np.array([0.8, 0.9, 0.93, 0.96, 0.97, 0.975, 0.99, 1.])
    levels = np.array([0.8, 0.9, 0.93, 0.96, 0.97, 0.975, 0.977, 0.9784])

    plt.figure(2)
    plt.contourf(omega_grid_plot, torque_grid_plot, efficiency_grid, cmap='jet', levels=levels_f)
    plt.colorbar()
    plt.clim(0,1)
    contours = plt.contour(omega_grid_plot, torque_grid_plot, efficiency_grid, colors='black', levels=levels)
    plt.clabel(contours, inline=True, fontsize=8)
    plt.plot(omega_range, upper_torque_curve, 'k', linewidth=5)
    plt.xlabel('RPM')
    plt.ylabel('EM Torque (Nm)')
    plt.title('Efficiency Map')
    plt.show()
