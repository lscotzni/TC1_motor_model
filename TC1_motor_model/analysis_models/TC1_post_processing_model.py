import numpy as np
import matplotlib.pyplot as plt

from csdl import Model
import csdl
from csdl_om import Simulator

class PostProcessingModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('op_voltage')
        self.parameters.declare('V_lim')
        self.parameters.declare('num_nodes')

    def define(self):

        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        num_nodes = self.parameters['num_nodes']

        ''' FLUX WEAKENING (I_d) & MTPA SMOOTHING (I_q) '''
        # convert I_d of flux weakening into I_q using torque equation
        Id_fw = self.declare_variable('Id_fw', shape=(num_nodes,1))
        L_q = self.declare_variable('L_q', shape=(num_nodes,1))
        L_d = self.declare_variable('L_d', shape=(num_nodes,1))
        phi = self.declare_variable('phi', shape=(num_nodes,1))
        T_em = self.declare_variable('T_em', shape=(num_nodes,1))

        Iq_fw = T_em / (1.5*p * (phi + (L_d-L_q)*Id_fw)) # CHECK SIZE OF COMPUTATIONS HERE

        # smoothing
        Iq_MTPA = self.declare_variable('Iq_MTPA', shape=(num_nodes,1)) # CHECK NAMING SCHEME FOR VARIABLE
        k = 1 # ASK SHUOFENG WHAT THIS IS
        I_q = (csdl.exp(k*(op_voltage - V_lim))*Iq_fw + csdl.exp(-1/(k*(op_voltage - V_lim)))*Iq_MTPA) / \
            (csdl.exp(k*(op_voltage - V_lim)) + csdl.exp(-1/(k*(op_voltage - V_lim))))

        # calculating I_d
        I_d = (T_em / (1.5*p*I_q) - phi) / (L_d-L_q) # CHECK SIZE OF COMPUTATIONS HERE

        ''' POWER LOSS CALCULATIONS '''
        torque = self.declare_variable('torque', shape=(num_nodes,1))
        omega = self.declare_variable('omega', shape=(num_nodes,1))
        # load power
        # eq of the form P0 = speed * torque
        P0 = torque * omega
        frequency = self.declare_variable('frequency', shape=(num_nodes,1))

        # copper loss
        R_dc = self.declare_variable('R_dc')
        P_copper = m*R_dc*csdl.sqrt(I_q**2 + I_d**2) # NEED TO CHECK IF THIS IS ELEMENT-WISE SQUARING

        # eddy_loss
        a = 0.00055 # lamination thickness in m
        sigma_c = 2e6 # bulk conductivity (2000000 S/m)
        l_ef = self.declare_variable('l_ef')
        D1 = self.declare_variable('outer_stator_radius')
        D_i = self.declare_variable('inner_stator_radius')
        Acu = self.declare_variable('Acu')
        B_delta = self.declare_variable('B_delta')
        
        K_e = (a*np.pi)**2 * sigma_c/6
        V_s = (np.pi*l_ef*(D1-D_i)^2)-36*l_ef*Acu; # volume of stator
        K_c = 0.822;
        P_eddy = K_e*V_s*(B_delta*frequency)**2; # eddy loss

        # hysteresis loss
        K_h = 100
        n_h = 2
        P_h = K_h*V_s*frequency*B_delta**n_h

        # stress loss
        P_stress = 0.01*P0

        # windage & friction loss
        k_r = 4 # roughness coefficient
        fr = 3e-3 # friction coefficient
        rho_air = 1.225 # air density kg/m^3
        D2 = self.declare_variable('rotor_radius')
        P_wo = k_r*np.pi*fr*rho_air*(2*np.pi*frequency)**2*l_ef*D2**4

        # total losses
        P_loss = P_copper + P_eddy + P_h + P_stress + P_wo
        efficiency = P0/(P0+P_loss)

        efficiency = self.register_output('efficency', efficiency)
        
        
''' ---- NOTES ----
- MTPA-Flux Weakening smoothing happens here

- we are calculating the power losses
    - copper, eddy, hysteresis, windage & friction
- use same methods from the high-fidelity modeling to calculate torque loss
- final output is the efficiency for the implicit motor analysis model

'''
