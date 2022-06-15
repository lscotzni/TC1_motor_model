import numpy as np
from csdl import Model, ScipyKrylov, NewtonSolver
from csdl_om import Simulator
import csdl

from analysis_models.TC1_magnet_mec_model import MagnetMECModel
from analysis_models.TC1_inductance_mec_model import InductanceModel
from analysis_models.TC1_flux_weakening_model import FluxWeakeningModel
from analysis_models.TC1_mtpa_model import MTPAModel
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
        self.parameters.declare('num_nodes')
        self.parameters.declare('op_voltage')

    def define(self):
        num_nodes = self.parameters['num_nodes']
        op_voltage = self.parameters['op_voltage']

        model=Model()
        T_load = model.declare_variable('T_load', shape=(num_nodes,1)) # comes from rotor model
        T_em = model.declare_variable('T_em', val=500, shape=(num_nodes,1))  # state of implicit model

        model.add(
            'magnet_MEC_model',
            MagnetMECModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage,
            )
        )

        model.add(
            'inductance_MEC_model',
            InductanceModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage,
            )
        )

        model.add(
            'flux_weakening_model',
            FluxWeakeningModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage,
            )
        )

        model.add(
            'mtpa_model',
            MTPAModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage,
            )
        )

        model.add(
            'post_processing_model',
            PostProcessingModel(
                num_nodes=num_nodes,
                op_voltage=op_voltage,
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

        T_load = self.declare_variable('T_load')
        efficiency = self.declare_variable('efficiency')
        T_em = self.declare_variable('T_em')
        # FROM HERE, THE OUTPUT POWER AND EFFICIENCY ARE PROPOGATED TO THE
        # BATTERY ANALYSIS MODELS

        




