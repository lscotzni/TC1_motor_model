import numpy as np 
from csdl import Model, GraphRepresentation
from python_csdl_backend import Simulator

class NodeModel(Model):
    def initialize(self):
        self.parameters.declare('instance')
        self.parameters.declare('num_nodes')
        self.parameters.declare('name')

        self.num_nodes = self.parameters['num_nodes']
        self.instance = self.parameters['instance']
        self.name = self.parameters['name']

''' NOTE:
    - NodeModel is good to inherit from because each of these is a CSDL model
    - there is a name and instance tied to each; assume nothing is promoted

'''

class DCBusModel(NodeModel):
    def initialize(self, kwargs):
        super().initialize(kwargs)
    
    def define(self):
        output_power_DCBus = self.declare_variable(
            ''
        )
    

class ConverterModel(NodeModel):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        self.parameters.declare('type', values=('acdc', 'dcdc'))

    def define(self):
        type = self.parameters['type']

class GearBoxModel(NodeModel):
    def initialize(self):
        super().initialize()
        self.parameters.declare('gear_ratio', types=int, default=4)

    def define(self):
        gear_ratio = self.parameters['gear_ratio']
        load_torque_rotor = self.declare_variable('load_torque_rotor')
        omega_rotor = self.declare_variable('omega_rotor')

        load_torque = self.register_output(
            'load_torque',
            load_torque_rotor/gear_ratio
        )
        omega = self.register_output(
            'omega',
            omega_rotor*gear_ratio
        )