import numpy as np
from csdl import Model, NewtonSolver, ScipyKrylov, GraphRepresentation
import csdl
from python_csdl_backend import Simulator

class MaxTorqueImplicitModel(Model):
    def define(self):
        A = self.declare_variable('A_quartic')
        B = self.declare_variable('B_quartic')
        C = self.declare_variable('C_quartic')
        D = self.declare_variable('D_quartic')
        E = self.declare_variable('E_quartic')

        T_lim = self.declare_variable('T_lim', val=10000) # STATE

        T_lim_residual = self.register_output(
            'T_lim_residual',
            # A/E*T_lim**4 + B/E*T_lim**3 + C/E*T_lim**2 + D/E*T_lim + 1,
            A/C*T_lim**4 + B/C*T_lim**3 + T_lim**2 + D/C*T_lim + E/C
        )

class MaxTorqueModel(Model):
    def initialize(self):
        self.parameters.declare('num_nodes')

    def define(self):
        num_nodes = self.parameters['num_nodes']

        A = self.declare_variable('A_quartic', shape=(num_nodes,))
        B = self.declare_variable('B_quartic', shape=(num_nodes,))
        C = self.declare_variable('C_quartic', shape=(num_nodes,))
        D = self.declare_variable('D_quartic', shape=(num_nodes,))
        E = self.declare_variable('E_quartic', shape=(num_nodes,))

        bracket_lower = self.declare_variable('max_cubic_root', shape=(num_nodes,))
        bracket_upper = self.declare_variable('upper_quartic_bracket', shape=(num_nodes,))
        self.register_output('dummy_output', bracket_upper/bracket_lower) # HERE TO CREATE PROPER ORDER OF OPERATIONS
        # bracket_lower = self.declare_variable('bracket_lower', 1600)
        # bracket_upper = self.declare_variable('bracket_upper', 2000)

        # max_torque_implicit_model = MaxTorqueImplicitModel()
        max_torque_implicit_model = csdl.Model()
        A = max_torque_implicit_model.declare_variable('A_quartic', shape=(num_nodes,))
        B = max_torque_implicit_model.declare_variable('B_quartic', shape=(num_nodes,))
        C = max_torque_implicit_model.declare_variable('C_quartic', shape=(num_nodes,))
        D = max_torque_implicit_model.declare_variable('D_quartic', shape=(num_nodes,))
        E = max_torque_implicit_model.declare_variable('E_quartic', shape=(num_nodes,))
        T_lim = max_torque_implicit_model.declare_variable('T_lim', shape=(num_nodes,)) # STATE

        T_lim_residual = max_torque_implicit_model.register_output(
            'T_lim_residual',
            # A/E*T_lim**4 + B/E*T_lim**3 + C/E*T_lim**2 + D/E*T_lim + 1,
            A/C*T_lim**4 + B/C*T_lim**3 + T_lim**2 + D/C*T_lim + E/C
        )

        max_torque_implicit_op = self.create_implicit_operation(max_torque_implicit_model)
        max_torque_implicit_op.declare_state(
            'T_lim',
            residual='T_lim_residual',
            bracket=(bracket_lower, bracket_upper)
            # bracket=(1600,2000)
        )
        max_torque_implicit_op.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=10000,
            iprint=True
        )
        max_torque_implicit_op.linear_solver = ScipyKrylov()

        A = self.declare_variable('A_quartic', shape=(num_nodes,))
        B = self.declare_variable('B_quartic', shape=(num_nodes,))
        C = self.declare_variable('C_quartic', shape=(num_nodes,))
        D = self.declare_variable('D_quartic', shape=(num_nodes,))
        E = self.declare_variable('E_quartic', shape=(num_nodes,))

        T_lim = max_torque_implicit_op(
            A, B, C, D, E
        )

class TorqueLimitModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']
        R = self.declare_variable('Rdc')
        Ld = self.declare_variable('L_d')
        Lq = self.declare_variable('L_q')
        omega = self.declare_variable('omega')
        phi = self.declare_variable('phi_air')

        R_expanded = csdl.expand(R, (num_nodes,))
        L_d_expanded = csdl.expand(Ld, (num_nodes,))
        L_q_expanded = csdl.expand(Lq, (num_nodes,))
        phi_air_expanded = csdl.expand(phi, (num_nodes,))

        # COMPILING COEFFICIENTS FROM ORIGINAL I_q FLUX WEAKENING EQUATION
        den = 3*p*(L_d_expanded-L_q_expanded)
        a = den**2*((omega*L_q_expanded)**2 + R**2)
        # c_1 and c_2 below make up coefficients for c = A*T + B
        c_1 = 12*p*omega*R_expanded*(L_d_expanded-L_q_expanded)**2 # labeled A in notes
        c_2 = (3*p*phi_air_expanded)**2*(R_expanded**2 + (omega*L_q_expanded)**2) - (V_lim*den)**2 # labeled B in notes
        d = -12*p*phi_air_expanded*(omega**2*L_d_expanded*L_q_expanded + R_expanded**2) # coefficient without torque
        e = 4*((omega*L_d_expanded)**2 + R_expanded**2) # coefficient without torque

        # COMBINED COEFFICIENTS FOR QUARTIC TORQUE EQUATION (DISCRIMINANT = 0 CASE)
        A = self.register_output('A_quartic', 256*a**2*e**3 - 128*a*e**2*c_1**2 + 16*e*c_1**4)
        B = self.register_output('B_quartic', -256*a*e**2*c_1*c_2 + 144*a*d**2*e*c_1 + 64*e*c_1**3*c_2 - 4*d**2*c_1**3)
        C = self.register_output('C_quartic', -128*a*e**2*c_2**2 + 144*a*d**2*e*c_2 - 27*a*d**4 + 96*c_1**2*c_2**2*e - 12*d**2*c_1**2*c_2)
        D = self.register_output('D_quartic', 64*e*c_1*c_2**3 - 12*d**2*c_1*c_2**2)
        E = self.register_output('E_quartic', 16*e*c_2**4 - 4*d**2*c_2**3)

        # DERIVATIVE OF ZERO DISCRIMINANT QUARTIC TORQUE EQ TO GET BRACKETS
        a_cubic = 3*B/(4*A)
        b_cubic = 2*C/(4*A)
        c_cubic = D/(4*A)

        P1 = (a_cubic**2 - 3*b_cubic)/9 # Q IN NOTES
        P2 = (2*a_cubic**3 - 9*a_cubic*b_cubic + 27*c_cubic)/54 # R IN NOTES
        theta = csdl.arccos(P2/(P1**3)**(1/2))

        # asdf = self.create_output('test_out', shape=(6,))
        # asdf[0] = a_cubic
        # asdf[1] = b_cubic
        # asdf[2] = c_cubic
        # asdf[3] = P1
        # asdf[4] = P2
        # asdf[5] = theta

        # NEED TO FIX SHAPES HERE
        cubic_roots = self.create_output('cubic_roots', shape=(3,))
        cubic_roots[0] = -1 * (2*(P1)**0.5*csdl.cos(theta/3)) - a_cubic/3
        cubic_roots[1] = -1 * (2*(P1)**0.5*csdl.cos((theta+2*np.pi)/3)) - a_cubic/3
        cubic_roots[2] = -1 * (2*(P1)**0.5*csdl.cos((theta-2*np.pi)/3)) - a_cubic/3

        max_cubic_root = csdl.max(cubic_roots)
        max_cubic_root = self.register_output(
            'max_cubic_root',
            csdl.max(cubic_roots) # LOWER END OF BRACKET FOR QUARTIC EQUATION
        )
        upper_quartic_bracket = max_cubic_root * 10.0
        upper_quartic_bracket = self.register_output(
            'upper_quartic_bracket',
            upper_quartic_bracket
        )

        self.add(MaxTorqueModel(num_nodes=num_nodes), 'max_torque_model')

if __name__ == '__main__':
    p = 6
    
    V_lim = 1000
    Rdc = 0.0313
    L_d = 0.0011
    L_q = 0.0022
    omega = 1100
    phi_air = 0.5494 # 0.0153 OR 0.5494

    # V_lim = 3000
    # Rdc = 0.091
    # L_d = 0.0043
    # L_q = 0.0126
    # omega = 2000
    # phi_air = 1.2952

    m = TorqueLimitModel(
        pole_pairs=p,
        V_lim=V_lim,
        num_nodes=1
    )

    rep = GraphRepresentation(m)

    sim = Simulator(rep)

    sim['Rdc'] = Rdc
    sim['L_d'] = L_d
    sim['L_q'] = L_q
    sim['omega'] = omega
    sim['phi_air'] = phi_air

    sim.run()
    # print('before visualize_implementation:')
    # print('torque: (found implicitly)', sim['T_lim'])
    # sim.visualize_implementation()
    # print('after visualize_implementation:')
    print('torque: (found implicitly)', sim['T_lim'])
    print(sim['max_cubic_root'])
    print(sim['upper_quartic_bracket'])
    print(sim['A_quartic'])
    print(sim['B_quartic'])
    print(sim['C_quartic'])
    print(sim['D_quartic'])
    print(sim['E_quartic'])
    print('------')
    print(sim['test_out'])
    print(sim['cubic_roots'])
