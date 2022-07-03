import numpy as np
from csdl import Model, NewtonSolver, ScipyKrylov
from csdl_om import Simulator

class MaxTorqueImplicitModelTest(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')

    def create_quartic_discriminant(self, a, b, c, d, e):
        self.quartic_discriminant = 256*a**3*e**3 - 192*a**2*b*d*e**2 - 128*a**2*c**2*e**2 \
            + 144*a**2*c*d**2*e - 27*a**2*d**4 + 144*a*b**2*c*e**2 - 6*a*b**2*d**2*e \
            - 80*a*b*c**2*d*e + 18*a*b*c*d**3 + 16*a*c**4*e - 4*a*c**3*d**2 - 27*b**4*e**2 \
            + 18*b**3*c*d*e - 4*b**3*d**3 - 4*b**2*c**3*e + b**2*c**2*d**2
        return self.quartic_discriminant

    def define(self):
        p = self.parameters['pole_pairs'] # pole pairs
        V_lim = self.parameters['V_lim'] # voltage limit
        R = self.declare_variable('Rdc')
        Ld = self.declare_variable('L_d')
        Lq = self.declare_variable('L_q')
        omega = self.declare_variable('omega')
        phi = self.declare_variable('phi_m')

        T_lim = self.declare_variable('T_lim', 1000) # STATE

        D = (3*p*(Ld-Lq))**2

        a = (3*R*p*(Ld-Lq))**2 + 9*p**2*(Ld*Lq-Lq**2)*omega**2
        b = 18*phi*p**2*R*(Ld-Lq)**2 * omega
        c = (3*p*phi*R)**2 - V_lim**2*D + (3*p*phi*Lq)**2*omega**2
        # b = 0
        # c = (3*p*phi)**2*(R**2+omega**2*Lq**2) + 12*T_lim*omega*p*R*(Ld-Lq)**2 - V_lim**2*D
        d = (-12*p*phi*(omega**2*Ld*Lq - R**2))/D
        e = 4*(R**2+omega**2*Ld**2)/D

        d1 = 256*(a*e)**3 # T^4
        d2 = -192*a**2*b*d*e**2 # T^3
        d3 = -128*(a*c*e)**2 # T^2
        d4 = 144*a**2*c*d**2*e # T^2
        d5 = -27*a**2*d**4 # T^2
        d6 = 144*a*b**2*c*e**2 # T^2
        d7 = -6*a*b**2*d**2*e # T^2
        d8 = -80*a*b*c**2*d*e # T^1
        d9 = 18*a*b*c*d**3 # T^1
        d10 = 16*a*c**4*e # T^0
        d11 = -4*a*c**3*d**2 # T^0
        d12 = -27*b**4*e**2 # T^2
        d13 = 18*b**3*c*d*e # T^1
        d14 = -4*b**3*d**3 # T^1
        d15 = -4*b**2*c**3*e # T^0
        d16 = b**2*c**2*d**2 # T^0

        quartic_torque_eq = d1*T_lim**4 + d2*T_lim**3 + (d3+d4+d5+d6+d7+d12)*T_lim**2 + \
            (d8+d9+d13+d14)*T_lim**1 + (d10+d11+d15+d16)

        T_lim_residual = self.register_output(
            'T_lim_residual',
            quartic_torque_eq
        )

class MaxTorqueTest(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        # R = self.declare_variable('Rdc')
        # Ld = self.declare_variable('L_d')
        # Lq = self.declare_variable('L_q')
        # omega = self.declare_variable('omega')
        # phi = self.declare_variable('phi_m')

        base_speed_implicit_model = MaxTorqueImplicitModelTest(
            pole_pairs=p,
            V_lim=V_lim
        )

        base_speed_implicit_op = self.create_implicit_operation(base_speed_implicit_model)
        base_speed_implicit_op.declare_state(
            'T_lim',
            residual='T_lim_residual',
            bracket=(0,5000)
        )
        base_speed_implicit_op.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=True
        )

        base_speed_implicit_op.linear_solver = ScipyKrylov()

        R = self.declare_variable('Rdc')
        Ld = self.declare_variable('L_d')
        Lq = self.declare_variable('L_q')
        omega = self.declare_variable('omega')
        phi = self.declare_variable('phi_m')

        T_lim= base_speed_implicit_op(
            R, Ld, Lq, omega, phi
        )


if __name__ == '__main__':
    p = 6
    V_lim = 2230
    m = MaxTorqueTest(
        pole_pairs=p,
        V_lim=V_lim
    )

    sim = Simulator(m)
    sim['Rdc'] = 0.091
    sim['L_d'] = 0.0043
    sim['L_q'] = 0.0126
    sim['omega'] = 1083
    sim['phi_m'] = 1.2952

    sim.run()
    print(sim['T_lim'])