import numpy as np
from csdl import Model, NewtonSolver, ScipyKrylov
# from csdl import GraphRepresentation
import csdl
from csdl_om import Simulator

from TC1_motor_model.motor_submodels.TC1_flux_weakening_model import FluxWeakeningModel
from TC1_motor_model.motor_submodels.TC1_mtpa_model import MTPAModel

class EMTorqueImplicitModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')
        self.parameters.declare('rated_current')
        self.parameters.declare('phases')
        self.parameters.declare('motor_variable_names')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']
        rated_current = self.parameters['rated_current']
        m = self.parameters['phases']

        self.motor_variable_names = self.parameters['motor_variable_names']

        T_em = self.declare_variable('T_em', shape=(num_nodes,)) # STATE of implicit model

        R_expanded = self.declare_variable('R_expanded', shape=(num_nodes,))
        L_d_expanded = self.declare_variable('L_d_expanded', shape=(num_nodes,))
        L_q_expanded = self.declare_variable('L_q_expanded', shape=(num_nodes,))
        PsiF_expanded = self.declare_variable('PsiF_expanded', shape=(num_nodes,))
        load_torque = self.declare_variable('load_torque', shape=(num_nodes,))
        omega = self.declare_variable('omega', shape=(num_nodes,))
        
        implicit_motor_variables = self.declare_variable('motor_variables', shape=(25,)) # array of motor sizing outputs
        for i in range(implicit_motor_variables.shape[0]):
            self.register_output(self.motor_variable_names[i], implicit_motor_variables[i])

        # FLUX WEAKENING MODEL
        self.add(
            FluxWeakeningModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            ),
            'flux_weakening_model',
        )

        # MTPA MODEL
        self.add(
            MTPAModel(
                pole_pairs=p,
                num_nodes=num_nodes
            ),
            'mtpa_model',
        )
        # POST-PROCESSING MODEL
        # self.add(
        #     PostProcessingModel(
        #         pole_pairs=p,
        #         phases=m,
        #         op_voltage=op_voltage,
        #         V_lim=V_lim,
        #         num_nodes=num_nodes
        #     ),
        #     'post_processing_model',
        # )
        I_q_rated = self.declare_variable('I_q_temp')
        I_q_rated_expanded = csdl.expand(I_q_rated, shape=(num_nodes,))

        f_i = 3000*p/60 # rated omega from sizing model = 3000
        U_d = -R_expanded*rated_current*np.sin(0.6283) - 2*np.pi*f_i*L_q_expanded*I_q_rated_expanded
        U_q = R_expanded*rated_current*np.sin(0.6283) + 2*np.pi*f_i*(PsiF_expanded - L_d_expanded*I_q_rated_expanded)
        U_rated = self.register_output(
            'voltage_amplitude',
            (U_d**2 + U_q**2)**(1/2)
        )

        Iq_fw = self.declare_variable('Iq_fw', shape=(num_nodes,))
        Iq_MTPA = self.declare_variable('Iq_MTPA', shape=(num_nodes,)) # CHECK NAMING SCHEME FOR VARIABLE
                    
        k = 1 # ASK SHUOFENG WHAT THIS IS
        I_q = (csdl.exp(k*(U_rated - V_lim))*Iq_fw + Iq_MTPA) / (csdl.exp(k*(U_rated - V_lim)) + 1.0)
        I_d = (T_em / (1.5*p*I_q) - PsiF_expanded) / (L_d_expanded-L_q_expanded) # CHECK SIZE OF COMPUTATIONS HERE
        current_amplitude = self.register_output(
            'current_amplitude',
            (I_q**2 + I_d**2)**0.5
        )
        ''' POWER LOSS CALCULATIONS '''
        # load power
        # eq of the form P0 = speed * torque
        P0 = load_torque * omega
        self.register_output('output_power', P0)
        frequency = omega*p/60

        # copper loss
        P_copper = m*R_expanded*current_amplitude**2 # NEED TO CHECK IF THIS IS ELEMENT-WISE SQUARING

        # eddy_loss
        a = 0.00055 # lamination thickness in m
        sigma_c = 2e6 # bulk conductivity (2000000 S/m)

        D_i = self.declare_variable('D_i')
        B_delta = self.declare_variable('B_delta')
        l_ef = implicit_motor_variables[4]
        D1 = implicit_motor_variables[0]
        Acu = implicit_motor_variables[7]
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
        # D2 = self.declare_variable('rotor_radius')
        D2 = implicit_motor_variables[5] # rotor radius
        l_ef_expanded = csdl.expand(l_ef, (num_nodes,))
        D2_expanded = csdl.expand(D2, (num_nodes,))
        P_wo = k_r*np.pi*fr*rho_air*(2*np.pi*frequency)**2*l_ef_expanded*D2_expanded**4

        # total losses
        P_loss = P_copper + P_eddy + P_h + P_stress + P_wo
        input_power = self.register_output('input_power', P0 + P_loss)
        efficiency = self.register_output('efficiency', P0/input_power)
        
        residual = self.register_output(
            'residual',
            load_torque - efficiency*T_em
        )


class EMTorqueModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')
        self.parameters.declare('rated_current')
        self.parameters.declare('phases')
        self.parameters.declare('motor_variable_names')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']
        rated_current = self.parameters['rated_current']
        m = self.parameters['phases']
        motor_var_names = self.parameters['motor_variable_names']
        T_lower_lim = self.declare_variable('T_lower_lim', val=0., shape=(num_nodes,))
        T_lim = self.declare_variable('T_lim', shape=(num_nodes,))

        model = EMTorqueImplicitModel(
            pole_pairs=p,
            V_lim=V_lim,
            num_nodes=num_nodes,
            rated_current=rated_current,
            phases=m,
            motor_variable_names=motor_var_names
        )

        implicit_torque_operation = self.create_implicit_operation(model)
        implicit_torque_operation.declare_state(
            state='T_em',
            residual='residual',
            bracket=(T_lower_lim, T_lim)
        )

        implicit_torque_operation.nonlinear_solver = NewtonSolver(
            solve_subsystems=True,
            maxiter=10,
            iprint=True,
        )
        implicit_torque_operation.linear_solver = ScipyKrylov()

        load_torque = self.declare_variable('load_torque', shape=(num_nodes,))
        omega = self.declare_variable('omega', shape=(num_nodes,))
        motor_variables = self.declare_variable('motor_variables', shape=(25,))
        R_expanded=self.declare_variable('R_expanded', shape=(num_nodes,))
        L_d_expanded = self.declare_variable('L_d_expanded', shape=(num_nodes,))
        L_q_expanded = self.declare_variable('L_q_expanded', shape=(num_nodes,))
        PsiF_expanded = self.declare_variable('PsiF_expanded', shape=(num_nodes,))
        I_q_rated = self.declare_variable('I_q_temp')
        B_delta = self.declare_variable('B_delta')
        D_i = self.declare_variable('D_i')

        T_em, efficiency, input_power, current_amplitude, output_power = implicit_torque_operation(
            load_torque, omega, motor_variables, R_expanded, L_d_expanded, L_q_expanded, 
            PsiF_expanded, I_q_rated, B_delta, D_i,
            expose=['efficiency', 'input_power', 'current_amplitude', 'output_power']
        )
        
if __name__ == '__main__':
    p = 6
    V_lim = 800
    num_nodes=1
    rated_current=123
    m=3
    motor_variable_names = [
        'outer_stator_radius', 'pole_pitch', 'tooth_pitch', 'air_gap_depth', 'l_ef',
        'rotor_radius', 'turns_per_phase', 'Acu',  'tooth_width', 'height_yoke_stator',
        'slot_bottom_width', 'slot_height', 'slot_width_inner', 'Tau_y', 'L_j1', 'Kdp1',
        'bm', 'Am_r', 'phi_r', 'lambda_m', 'alpha_i', 'Kf', 'K_phi', 'K_theta', 'A_f2',
    ]
    
    model = EMTorqueModel(
        pole_pairs=p,
        V_lim=V_lim,
        num_nodes=num_nodes,
        rated_current=rated_current,
        phases=m,
        motor_variable_names=motor_variable_names
    )
    # rep = GraphRepresentation(model)
    sim = Simulator(model)
    sim.visualize_implementation()
    # sim['T_lim']
    # sim['load_torque']
    # sim['omega']
