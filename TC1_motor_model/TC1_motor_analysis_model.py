import numpy as np
from csdl import Model, ScipyKrylov, NewtonSolver
from csdl_om import Simulator
import csdl

from TC1_motor_model.analysis_models.TC1_upper_limit_torque_model import UpperLimitTorqueModel
from TC1_motor_model.analysis_models.TC1_magnet_mec_model import MagnetMECModel
from TC1_motor_model.analysis_models.TC1_inductance_mec_model import InductanceModel
from TC1_motor_model.analysis_models.TC1_flux_weakening_model import FluxWeakeningModel
from TC1_motor_model.analysis_models.TC1_mtpa_model import MTPAModel
from TC1_motor_model.analysis_models.TC1_post_processing_model import PostProcessingModel

'''
THIS MODEL CONTAINS THE ENTIRE MOTOR ANALYSIS MODEL.
THE GOAL OF THIS MODEL IS TO IMPLICITLY SOLVE FOR POWER AND EFFICIENCY
GIVEN A RESIDUAL CONSIDERING TORQUE AND EFFICIENCY.

FEOC = FOR EACH OPERATING CONDITION
INPUTS:
    - motor geometry (from sizing model)
    - RPM and load torque (FEOC)
    - voltage limit
    - voltage (FEOC); fed back in from the battery model

OUTPUTS:
    - power and efficiency (FEOC); fed into other models
    - EM torque (FEOC)
'''

class TC1MotorAnalysisModel(Model):
    def initialize(self):
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('num_slots') # 36
        self.parameters.declare('op_voltage')
        self.parameters.declare('V_lim')
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H, B = f(H))
        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B, H = g(B))
        self.parameters.declare('num_nodes')

    def define(self):
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        op_voltage = self.parameters['op_voltage']
        V_lim = self.parameters['V_lim']
        fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']
        num_nodes = self.parameters['num_nodes']
        
        omega = self.declare_variable('omega', shape=(num_nodes,1))
        load_torque = self.declare_variable('load_torque', shape=(num_nodes,1))
        motor_variables = self.declare_variable('motor_variables', shape=(23,1)) # array of motor sizing outputs

        self.register_output('outer_stator_radius', motor_variables[0,0])

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
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            ),
            'inductance_MEC_model',
        )

        self.add(
            UpperLimitTorqueModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                num_nodes=num_nodes
            ),
            'max_torque_model'
        )

        model=Model()
        load_torque = model.declare_variable('load_torque', shape=(num_nodes,1)) # comes from rotor model
        omega = model.declare_variable('omega', shape=(num_nodes,1))
        em_torque = model.declare_variable('em_torque', val=500, shape=(num_nodes,1))  # state of implicit model
        model.declare_variable('Rdc', val=10) # DC resistance
        model.declare_variable('motor_variables', shape=(23,1)) # array of motor sizing outputs

        # declare variables here
        

        # BETTER METHOD:
        # THE MODELS THAT GET ADDED HERE ONLY DO THE IMPLICIT OPERATIONS; ANY SUBSEQUENT POST-PROCESSING WILL OCCUR
        # IN THIS PART OF THE MODEL, SO THAT OUTPUTS CAN BE REGISTERED

        model.add(
            FluxWeakeningModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            ),
            'flux_weakening_model',
        )

        model.add(
            MTPAModel(
                pole_pairs=p,
                num_nodes=num_nodes
            ),
            'mtpa_model',
        )

        model.add(
            PostProcessingModel(
                pole_pairs=p,
                phases=m,
                op_voltage=op_voltage,
                V_lim=V_lim,
                num_nodes=num_nodes
            ),
            'post_processing_model',
        )

        efficiency_pp = model.declare_variable('efficiency_pp', shape=(num_nodes,1)) # comes from post-processing
        efficiency = model.register_output('efficiency', 1*efficiency_pp)
        input_power_pp = model.declare_variable('input_power_pp', shape=(num_nodes,1)) # comes from post-processing
        input_power = model.register_output('input_power', 1*input_power_pp)
        
        residual = model.register_output(
            'residual',
            load_torque - efficiency*em_torque
        )

        solve_motor_analysis = self.create_implicit_operation(model)
        solve_motor_analysis.declare_state(
            state='em_torque',
            residual='residual',
            # not sure if bracket is necessary
        )

        solve_motor_analysis.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=False,
        )
        solve_motor_analysis.linear_solver = ScipyKrylov()

        load_torque = self.declare_variable('load_torque', shape=(num_nodes,1))
        Rdc = self.declare_variable('Rdc', val=5) # DC resistance
        motor_variables = self.declare_variable('motor_variables', shape=(23,1)) # array of motor sizing outputs

        T_em, efficiency, input_power = solve_motor_analysis(load_torque, Rdc, motor_variables, 
            expose=['efficiency', 'input_power']
        )
        # FROM HERE, THE OUTPUT POWER AND EFFICIENCY ARE PROPOGATED TO THE
        # BATTERY ANALYSIS MODELS

        




