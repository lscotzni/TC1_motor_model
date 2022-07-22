import numpy as np 
from csdl import Model, GraphRepresentation
from csdl_om import Simulator

from powertrain_submodels import DCBusModel, ConverterModel

class PowertrainModel(Model):
    def initialize(self):
        self.parameters.declare('num_motors')
        self.parameters.declare('num_acdc_converters')
        self.parameters.declare('num_dc_buses')
        self.parameters.declare('num_dcdc_converters')
        self.parameters.declare('num_batteries')
        self.parameters.declare('num_modes')

    def define(self):
        num_motors = self.parameters['num_motors']
        num_acdc = self.parameters['num_acdc_converters']
        num_dc_bus = self.parameters['num_dc_buses']
        num_dcdc = self.parameters['num_dcdc_converters']
        num_batt = self.parameters['num_batteries']
        num_nodes = self.parameters['num_nodes']

        motor_input_power = self.declare_variable(
            'motor_input_power',
            shape=(num_nodes,)
        )

        motor_input_current = self.declare_variable(
            'motor_input_current',
            shape=(num_nodes,)
        )

        motor_input_voltage = self.declare_variable(
            'motor_input_voltage',
            shape=(num_nodes,)
        )