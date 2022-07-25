import numpy as np
import matplotlib.pyplot as plt

from csdl import Model, ScipyKrylov, NewtonSolver
import csdl
from csdl_om import Simulator

class FluxWeakeningBracketModel(Model):
    def initialize(self):
        self.parameters.declare('num_nodes')
        self.parameters.declare('pole_pairs') # 6

    def define(self):
        num_nodes = self.parameters['num_nodes']
        p = self.parameters['pole_pairs']
        # a_bracket = self.declare_variable('a_bracket', shape=(num_nodes, ))
        # c_bracket = self.declare_variable('c_bracket', shape=(num_nodes, ))
        # d_bracket = self.declare_variable('d_bracket', shape=(num_nodes, ))
        # e_bracket = self.declare_variable('e_bracket', shape=(num_nodes, ))

        # --------------------------------- IMPLICIT MODEL START ---------------------------------
        bracket_implicit_model = Model()
        a_bracket = bracket_implicit_model.declare_variable('a_bracket', shape=(num_nodes, ))
        c_bracket = bracket_implicit_model.declare_variable('c_bracket', shape=(num_nodes, ))
        d_bracket = bracket_implicit_model.declare_variable('d_bracket', shape=(num_nodes, ))
        e_bracket = bracket_implicit_model.declare_variable('e_bracket', shape=(num_nodes, ))

        Iq_fw_bracket = bracket_implicit_model.declare_variable('I_q_fw_bracket', shape=(num_nodes, )) # STATE
        Iq_fw_bracket_res = bracket_implicit_model.register_output(
            'Iq_fw_bracket_res',
            a_bracket/e_bracket*Iq_fw_bracket**4 + c_bracket/e_bracket*Iq_fw_bracket**2 + \
                d_bracket/e_bracket*Iq_fw_bracket + 1
        )
        # --------------------------------- IMPLICIT MODEL END ---------------------------------

        Iq_fw_bracket_implicit_op = self.create_implicit_operation(bracket_implicit_model)
        Iq_fw_bracket_implicit_op.declare_state(
            'I_q_fw_bracket',
            residual='Iq_fw_bracket_res'
        )
        Iq_fw_bracket_implicit_op.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=10000,
            iprint=True
        )
        Iq_fw_bracket_implicit_op.linear_solver = ScipyKrylov()

        a_bracket = self.declare_variable('a_bracket', shape=(num_nodes, ))
        c_bracket = self.declare_variable('c_bracket', shape=(num_nodes, ))
        d_bracket = self.declare_variable('d_bracket', shape=(num_nodes, ))
        e_bracket = self.declare_variable('e_bracket', shape=(num_nodes, ))

        Iq_fw_bracket = Iq_fw_bracket_implicit_op(
            a_bracket, c_bracket, d_bracket, e_bracket
        )

        L_d_expanded = self.declare_variable('L_d_expanded', shape=(num_nodes,))
        L_q_expanded = self.declare_variable('L_q_expanded', shape=(num_nodes,))
        PsiF_expanded = self.declare_variable('PsiF_expanded', shape=(num_nodes,))
        T_lim = self.declare_variable('T_lim', shape=(num_nodes,)) # UPPER CURVE LIMIT

        Id_fw_bracket = self.register_output(
            'Id_fw_bracket',
            (2*T_lim/(3*p*Iq_fw_bracket) - PsiF_expanded) / (L_d_expanded-L_q_expanded)
        )

class FluxWeakeningModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']
        
        omega = self.declare_variable('omega', shape=(num_nodes,))
        T_em = self.declare_variable('T_em', shape=(num_nodes,))
        T_lim = self.declare_variable('T_lim', shape=(num_nodes,)) # UPPER CURVE LIMIT

        # R_expanded = self.register_output('R_expanded', csdl.expand(R, (num_nodes,)))
        # L_d_expanded = self.register_output('L_d_expanded', csdl.expand(L_d, (num_nodes,)))
        # L_q_expanded = self.register_output('L_q_expanded',csdl.expand(L_q, (num_nodes,)))
        # PsiF_expanded = self.register_output('PsiF_expanded',csdl.expand(phi_air, (num_nodes,)))

        R_expanded = self.declare_variable('R_expanded', shape=(num_nodes,))
        L_d_expanded = self.declare_variable('L_d_expanded', shape=(num_nodes,))
        L_q_expanded = self.declare_variable('L_q_expanded', shape=(num_nodes,))
        PsiF_expanded = self.declare_variable('PsiF_expanded', shape=(num_nodes,))

        D = (3*p*(L_d_expanded-L_q_expanded))
        if False:
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
                FluxWeakeningBracketModel(pole_pairs=p, num_nodes=num_nodes),
                'flux_weakening_bracket_method'
            )
        Id_fw_bracket_low = self.declare_variable('Id_fw_bracket', shape=(num_nodes,))
        # THE LOWER END OF THE BRACKET (MOST NEGATIVE, WHERE DISCRIMINANT = 0)
        
        # FLUX WEAKENING IMPLICIT MODEL
        a1 = self.register_output(
            'a1',
            D**2 * (R_expanded**2 + (omega*L_d_expanded)**2),
        )

        a2 = self.register_output(
            'a2',
            18*p**2*PsiF_expanded*(L_d_expanded-L_q_expanded) * \
                (R_expanded**2 + omega**2*L_d_expanded*(2*L_d_expanded-L_q_expanded))
        )

        a3 = self.register_output(
            'a3',
            (3*p*PsiF_expanded)**2*(R_expanded**2 + (omega*(2*L_d_expanded-L_q_expanded))**2) - \
            (3*p*V_lim*(L_d_expanded-L_q_expanded))**2 + 6*p*omega*(L_d_expanded-L_q_expanded) * \
                (3*PsiF_expanded**2*p*omega*L_d_expanded + 2*R_expanded*T_em*(L_d_expanded-L_q_expanded))
        )

        a4 = self.register_output(
            'a4',
            6*p*omega*PsiF_expanded*(3*PsiF_expanded**2*p*omega*(2*L_d_expanded-L_q_expanded) + 4*R_expanded*T_em*(L_d_expanded-L_q_expanded)) - \
                18*(p*V_lim)**2*PsiF_expanded*(L_d_expanded-L_q_expanded)
        )

        a5 = self.register_output(
            'a5',
            (2*T_em*L_q_expanded*omega)**2 + (3*PsiF_expanded**2*p*omega)**2 + (2*R_expanded*T_em)**2 + \
                12*PsiF_expanded**2*p*omega*R_expanded*T_em - (3*p*V_lim*PsiF_expanded)**2
        )
        '''
        BRACKET FOR IMPLICIT FLUX WEAKENING MODEL
        '''
        I_d_asymp = self.register_output(
            'I_d_asymp',
            -PsiF_expanded/(L_d_expanded - L_q_expanded)
        ) # UPPER LIMIT OF I_d WHERE I_q ASYMPTOTES TO INFINITY

        
        ''' --- START IMPLICIT MODEL FOR FLUX WEAKENING --- '''
        self.add(
            FluxWeakeningImplicitModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            )
        )

class FluxWeakeningImplicitModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')

    def define(self):
        p = self.parameters['pole_pairs']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']

        L_d_expanded = self.declare_variable('L_d_expanded', shape=(num_nodes,))
        L_q_expanded = self.declare_variable('L_q_expanded', shape=(num_nodes,))
        PsiF_expanded = self.declare_variable('PsiF_expanded', shape=(num_nodes,))
        T_em = self.declare_variable('T_em', shape=(num_nodes,))

        I_d_asymp =  self.declare_variable('I_d_asymp', shape=(num_nodes,))
        Id_fw_bracket_low = self.declare_variable('Id_fw_bracket', shape=(num_nodes,))

        self.register_output('dummy_out_FW', Id_fw_bracket_low/I_d_asymp)

        ''' --- START IMPLICIT MODEL FOR FLUX WEAKENING --- '''
        model=Model()

        Id_fw = model.declare_variable('Id_fw', shape=(num_nodes,)) # STATE

        a1 = model.declare_variable('a1', shape=(num_nodes,))
        a2 = model.declare_variable('a2', shape=(num_nodes,))
        a3 = model.declare_variable('a3', shape=(num_nodes,))
        a4 = model.declare_variable('a4', shape=(num_nodes,))
        a5 = model.declare_variable('a5', shape=(num_nodes,))

        residual = model.register_output(
            'Id_fw_residual',
            a1*Id_fw**4 + a2*Id_fw**3 + a3*Id_fw**2 + a4*Id_fw + a5
        )
        ''' --- END IMPLICIT MODEL FOR FLUX WEAKENING --- '''

        solve_flux_weakening = self.create_implicit_operation(model)
        solve_flux_weakening.declare_state('Id_fw', 
            residual='Id_fw_residual', 
            bracket=(Id_fw_bracket_low, I_d_asymp),
            # bracket=(-587.9010, 13.90909091)
        )
        solve_flux_weakening.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=True,
        )
        solve_flux_weakening.linear_solver = ScipyKrylov()

        a1 = self.declare_variable('a1', shape=(num_nodes,))
        a2 = self.declare_variable('a2', shape=(num_nodes,))
        a3 = self.declare_variable('a3', shape=(num_nodes,))
        a4 = self.declare_variable('a4', shape=(num_nodes,))
        a5 = self.declare_variable('a5', shape=(num_nodes,))

        Id_fw = solve_flux_weakening(a1, a2, a3, a4, a5)

        Iq_fw = self.register_output(
            'Iq_fw',
            T_em/(1.5*p)/(PsiF_expanded+(L_d_expanded-L_q_expanded)*Id_fw)
        )


''' 
TO-DO: 
    - ADD BRACKETS IN TERMS OF CSDL VARIABLES (comment out but have the form of the equation ready)



'''

if __name__ == '__main__':
    m = FluxWeakeningModel(
        pole_pairs=6,
        V_lim=1000,
        num_nodes=1,
    )

    # rep = csdl.GraphRepresentation(m)
    sim = Simulator(m)

    sim['Rdc'] = 0.0313
    sim['L_d'] = 0.0011
    sim['L_q'] = 0.0022
    sim['phi_air'] = 0.0153
    sim['omega'] = 1100
    sim['T_em'] = 1000

    sim.run()
    print('I_d: ', sim['Id_fw'])
    print('I_q: ', sim['Iq_fw'])
    print('I_d bracket 1: ', sim['I_d_asymp'])
    print('I_d bracket 2: ', sim['I_d_hat'])
    print('I_q_hat: ', sim['I_q_hat'])
    print('p & q: ', sim['p_q'])
    print('condition: ', sim['cond'])