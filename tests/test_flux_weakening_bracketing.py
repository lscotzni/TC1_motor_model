import numpy as np
from csdl import GraphRepresentation, Model
from csdl_om import Simulator

from test_max_torque_model import *
from TC1_motor_model.motor_submodels.TC1_flux_weakening_model import *

class FluxWeakeningFullTest(Model):
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
        omega = self.declare_variable('omega', shape=(num_nodes,))
        phi = self.declare_variable('phi_air')

        T_em = self.declare_variable('T_em', shape=(num_nodes,))

        self.add(
            TorqueLimitModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            )
        )
        T_lim = self.declare_variable('T_lim', shape=(num_nodes,)) # UPPER CURVE LIMIT
        T_lim_bracket = self.register_output(
            'T_lim_bracket',
            T_lim * 1.0
        )

        R_expanded = csdl.expand(R, (num_nodes,))
        L_d_expanded = csdl.expand(Ld, (num_nodes,))
        L_q_expanded = csdl.expand(Lq, (num_nodes,))
        phi_air_expanded = csdl.expand(phi, (num_nodes,))

        # I_d_asymp = self.register_output(
        #     'I_d_asymp',
        #     -phi_air_expanded/(L_d_expanded - L_q_expanded)
        # ) # UPPER LIMIT OF I_d WHERE I_q ASYMPTOTES TO INFINITY
        

        self.add(
            FluxWeakeningModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            )
        )


if __name__ == '__main__':

    p = 6
    
    # V_lim = 1000
    # Rdc = 0.0313
    # L_d = 0.0011
    # L_q = 0.0022
    # omega = 1100
    # phi_air = 0.5494

    V_lim = 3000
    Rdc = 0.091
    L_d = 0.0043
    L_q = 0.0126
    omega = 2500
    phi_air = 1.2952

    m = FluxWeakeningFullTest(
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
    sim['T_em'] = 3000
    # sim['T_lim'] = 1700

    sim.visualize_implementation()
    # sim.run()

    print('Max torque coeff A: ', sim['A_quartic'])
    print('Max torque coeff B: ', sim['B_quartic'])
    print('Max torque coeff C: ', sim['C_quartic'])
    print('Max torque coeff D: ', sim['D_quartic'])
    print('Max torque coeff E: ', sim['E_quartic'])


    print('Iq bracket coeff a: ', sim['a_bracket'])
    print('Iq bracket coeff c: ', sim['c_bracket'])
    print('Iq bracket coeff d: ', sim['d_bracket'])
    print('Iq bracket coeff e: ', sim['e_bracket'])

    print('Lower Id bracket from Iq implicit op: ', sim['Id_fw_bracket'])

    print('Upper Id bracket from asymptotic behavior: ', sim['I_d_asymp'])
    
    print('Id FW coeff a1: ', sim['a1'])
    print('Id FW coeff a2: ', sim['a2'])
    print('Id FW coeff a3: ', sim['a3'])
    print('Id FW coeff a4: ', sim['a4'])
    print('Id FW coeff a5: ', sim['a5'])

    print('Flux Weakening Iq: ', sim['Iq_fw'])
    print('Flux Weakening Id: ', sim['Id_fw'])
    print('Torque limit: ', sim['T_lim'])
    