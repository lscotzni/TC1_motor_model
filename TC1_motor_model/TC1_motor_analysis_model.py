import numpy as np
from csdl import Model, ScipyKrylov, NewtonSolver
from csdl_om import Simulator
import csdl

from analysis_models.TC1_magnet_mec_model import MagnetMECModel
from analysis_models.TC1_inductance_mec_model import InductanceModel
from analysis_models.TC1_flux_weakening_model import FluxWeakeningModel
from analysis_models.TC1_mtpa_model_new import MTPAModel
from analysis_models.TC1_post_processing_model import PostProcessingModel


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

        model=Model()
        load_torque = model.declare_variable('load_torque', shape=(num_nodes,1)) # comes from rotor model
        em_torque = model.declare_variable('em_torque', val=500, shape=(num_nodes,1))  # state of implicit model

        model.add(
            'magnet_MEC_model',
            MagnetMECModel(
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            )
        )

        model.add(
            'inductance_MEC_model',
            InductanceModel(
                pole_pairs=p,
                phases=m,
                num_slots=Z,
                fit_coeff_dep_H=fit_coeff_dep_H,
                fit_coeff_dep_B=fit_coeff_dep_B,
            )
        )

        model.add(
            'flux_weakening_model',
            FluxWeakeningModel(
                pole_pairs=p,
                V_lim=V_lim,
                num_nodes=num_nodes
            )
        )

        model.add(
            'mtpa_model',
            MTPAModel(
                pole_pairs=p,
                num_nodes=num_nodes
            )
        )

        model.add(
            'post_processing_model',
            PostProcessingModel(
                pole_pairs=p,
                phases=m,
                op_voltage=op_voltage,
                V_lim=V_lim,
                num_nodes=num_nodes
            )
        )

        efficiency = model.declare_variable('efficiency', shape=(num_nodes,1)) # comes from post-processing
        
        residual = model.register_output(
            'residual',
            T_load - efficiency*T_em
        )

        solve_motor_analysis = self.create_implicit_operation(model)
        solve_motor_analysis.declare_state(
            state='T_em',
            residual='residual',
            # not sure if bracket is necessary
        )

        solve_motor_analysis.nonlinear_solver = NewtonSolver(
            solve_subsystems=False,
            maxiter=100,
            iprint=False,
        )
        solve_motor_analysis.linear_solver = ScipyKrylov()

        T_load = self.declare_variable('T_load', shape=(num_nodes,1))
        efficiency = self.declare_variable('efficiency', shape=(num_nodes,1))
        T_em = solve_motor_analysis(T_load, efficiency)
        # FROM HERE, THE OUTPUT POWER AND EFFICIENCY ARE PROPOGATED TO THE
        # BATTERY ANALYSIS MODELS

        




