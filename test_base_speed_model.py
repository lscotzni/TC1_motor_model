import numpy as np
from csdl import Model, NewtonSolver, ScipyKrylov
from csdl_om import Simulator

class BaseSpeedImplicitModelTest(Model):
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
        T_max = self.declare_variable('T_max')
        phi = self.declare_variable('phi_m')

        base_speed = self.declare_variable('base_speed', val=1000) # STATE

        D = (3*p*(Ld-Lq))**2

        a = (3*R*p*(Ld-Lq))**2 + 9*p**2*(Ld*Lq-Lq**2)*base_speed**2
        b = 18*phi*p**2*R*(Ld-Lq)**2 * base_speed
        c = (3*p*phi*R)**2 - V_lim**2*D + (3*p*phi*Lq)**2*base_speed**2
        d = 12*p*phi*T_max*R**2 - 12*p*phi*T_max*Ld*Lq*base_speed**2
        e = (2*R*T_max)**2 + (2*Ld*T_max)**2*base_speed**2

        self.create_quartic_discriminant(a, b, c, d, e)

        base_speed_residual = self.register_output(
            'base_speed_residual',
            self.quartic_discriminant
        )

class BaseSpeedModelTest(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs')
        self.parameters.declare('V_lim')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        # R = self.declare_variable('Rdc')
        # Ld = self.declare_variable('L_d')
        # Lq = self.declare_variable('L_q')
        # T_max = self.declare_variable('T_max')
        # phi = self.declare_variable('phi_m')

        base_speed_implicit_model = BaseSpeedImplicitModelTest(
            pole_pairs=p,
            V_lim=V_lim
        )

        base_speed_implicit_op = self.create_implicit_operation(base_speed_implicit_model)
        base_speed_implicit_op.declare_state(
            'base_speed',
            residual='base_speed_residual',
            # bracket=(0,10000)
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
        T_max = self.declare_variable('T_max')
        phi = self.declare_variable('phi_m')

        base_speed = base_speed_implicit_op(
            R, Ld, Lq, T_max, phi
        )


if __name__ == '__main__':
    p = 3
    V_lim = 2230
    m = BaseSpeedModelTest(
        pole_pairs=p,
        V_lim=V_lim
    )

    sim = Simulator(m)
    sim['Rdc'] = 0.091
    sim['L_d'] = 0.0043
    sim['L_q'] = 0.0126
    sim['T_max'] = 3600
    sim['phi_m'] = 1.2952

    sim.run()
    print(sim['base_speed'])