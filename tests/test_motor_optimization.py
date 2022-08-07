import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from python_csdl_backend import Simulator
from csdl import Model, GraphRepresentation
from modopt.csdl_library import CSDLProblem
from modopt.scipy_library import SLSQP
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
        self.parameters.declare('num_active_nodes')
        self.parameters.declare('model_test', default=False)

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
        num_active_nodes = self.parameters['num_active_nodes']
        model_test = self.parameters['model_test']

        D_i = self.create_input('D_i', val=0.1) # inner radius of stator
        L = self.create_input('L', val=0.08) # effective length of motor

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
                num_active_nodes=num_active_nodes,
                model_test=model_test
            ),
            'TC1_motor_analysis_model',
        )

        eff_ac = self.declare_variable('efficiency_active', shape=(num_active_nodes,))
        self.print_var(var=eff_ac)
        self.print_var(var=D_i)
        self.print_var(var=L)

        self.add_design_variable('D_i', lower=0.08, upper=0.35)
        self.add_design_variable('L', lower=0.08, upper=0.35)

        self.add_objective('efficiency_active', scaler=-1.)



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
    V_lim = 800
    rated_current = 123

    # D_i = 0.3723
    # L = 0.2755

    # D_i = 0.182
    # L = 0.086

    load_torque_rotor = np.array([501.062])
    omega_rotor = np.array([3387.3981])

    num_nodes = len(load_torque_rotor)
    num_active_nodes = np.count_nonzero(load_torque_rotor)

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
        num_active_nodes=num_active_nodes,
        model_test=False,
    )

    rep = GraphRepresentation(m)
    sim = Simulator(rep)

    sim['omega_rotor'] = omega_rotor
    sim['load_torque_rotor'] = load_torque_rotor

    prob = CSDLProblem(
        problem_name='motor',
        simulator=sim,
    )

    # Setup your preferred optimizer (SLSQP) with the Problem object 
    # Pass in the options for your chosen optimizer
    optimizer = SLSQP(prob, maxiter=20)

    # Check first derivatives at the initial guess, if needed
    optimizer.check_first_derivatives(prob.x0)

    # Solve your optimization problem
    optimizer.solve()

    # Print results of optimization
    optimizer.print_results()