import numpy as np 
from csdl import Model, GraphRepresentation
from csdl_om import Simulator

class NodeModel(Model):
    def initialize(self):
        self.parameters.declare('instance')
        self.parameters.declare('num_nodes')
        self.parameters.declare('name')

        self.num_nodes = self.parameters['num_nodes']
        self.instance = self.parameters['instance']
        self.name = self.parameters['name']

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