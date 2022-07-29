import numpy as np 
from csdl import Model, GraphRepresentation
from csdl_om import Simulator

from powertrain_submodels import DCBusModel, ConverterModel

class PowertrainModel(Model):
    def initialize(self):
        self.parameters.declare('list_motors')
        self.parameters.declare('list_acdc_converters')
        self.parameters.declare('list_dc_buses')
        self.parameters.declare('list_dcdc_converters')
        self.parameters.declare('list_batteries')
        self.parameters.declare('num_nodes')
        self.parameters.declare('connection_list') # SPECIFIES INTERNAL NDOE CONNECTIONS FOR POWER

    def define(self):
        list_motors = self.parameters['list_motors']
        list_acdc = self.parameters['list_acdc_converters']
        list_dcbus = self.parameters['list_dc_buses']
        list_dcdc = self.parameters['list_dcdc_converters']
        list_batt = self.parameters['list_batteries']
        num_nodes = self.parameters['num_nodes']
        connection_list = self.parameters['connection_list']

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

        for i in range(list_acdc):
            self.add(
                ConverterModel(
                    instance=i+1,
                    num_nodes=num_nodes,
                    type='acdc'
                ),
                list_acdc[i] + '_{}'.format(i+1), # CHANGE, THE MODELS WILL ALREADY HAVE NAME AND ID
                promotes=['*']
            )

        for i in range(list_dcbus):
            self.add(
                DCBusModel(
                    instance=i+1,
                    num_nodes=num_nodes,
                ),
                'dcbus_model_{}'.format(i+1),
                promotes=['*']
            )

        for i in range(list_dcdc):
            self.add(
                ConverterModel(
                    instance=i+1,
                    num_nodes=num_nodes,
                    type='dcdc'
                ),
                'dcdc_model_{}'.format(i+1),
                promotes=['*']
            )

        # SPECIFY CONNECTIONS HERE