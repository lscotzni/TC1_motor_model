import numpy as np 
import matplotlib.pyplot as plt
from python_csdl_backend import Simulator
from python_csdl_backend import Simulator
from csdl import Model, GraphRepresentation
import csdl
import matplotlib.pyplot as plt

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
        V_lim = self.parameters['V_lim']
        rated_current = self.parameters['rated_current']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']
        num_active_nodes = self.parameters['num_active_nodes']
        model_test = self.parameters['model_test']

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
    V_lim = 800
    rated_current = 123

    load_torque_rotor = np.array([501.062])
    omega_rotor = np.array([3387.3981])

    num_nodes = len(load_torque_rotor)
    num_active_nodes = np.count_nonzero(load_torque_rotor)

    D_i_vec = np.linspace(0.08, 0.35, 15)
    L_vec = np.linspace(0.08, 0.35, 15)

    motor_mass_map = np.zeros((len(D_i_vec), len(L_vec)))
    efficiency_map = np.zeros(shape=motor_mass_map.shape)

    for i, D_i in enumerate(D_i_vec):
        for j, L in enumerate(L_vec):
            mm = TC1MotorModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                V_lim=V_lim,
                rated_current=rated_current,
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
                num_nodes=num_nodes,
                num_active_nodes=num_active_nodes,
                model_test=False,
            )
            rep = GraphRepresentation(mm)
            sim = Simulator(rep)
            sim['omega_rotor'] = omega_rotor
            sim['load_torque_rotor'] = load_torque_rotor
            sim['D_i'] = D_i
            sim['L'] = L
            sim.run()

            motor_mass_map[i,j] = sim['motor_mass']
            efficiency_map[i,j] = sim['efficiency_active']

    fig1, ax1 = plt.subplots()
    CS = ax1.contour(D_i_vec, L_vec, motor_mass_map)
    ax1.clabel(CS, fontsize=9, inline=True)
    plt.xlabel('Diameter')
    plt.ylabel('Length')
    plt.title('Motor Mass (kg)')
    plt.savefig('geom_sweep_results/motor_mass_map_geom_sweep.png')

    fig2, ax2 = plt.subplots()
    CS = ax2.contour(D_i_vec, L_vec, efficiency_map)
    ax2.clabel(CS, fontsize=9, inline=True)
    plt.xlabel('Diameter')
    plt.ylabel('Length')
    plt.title('Efficiency')
    plt.savefig('geom_sweep_results/efficiency_map_geom_sweep.png')

    plt.show()