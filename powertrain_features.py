import numpy as np
import csdl

from src.caddee.concept.feature import Feature
from src.utils.base_class import CsdlInputVariableInfo
from src.utils.base_class import BaseClass
from powertrain_model_orig import SimplePowertrainModel

from collections import OrderedDict

#  MotorFeature, BatteryFeature, ConverterFeature, DCBusFeature

class PowertrainComponent(Feature):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        self.parameters.declare('name', types=str)
        self.parameters.declare('output_branches', default=1, types=int)
        self.parameters.declare('efficiency', default=1.0, lower=0.0, upper=1.0)
        ''' --- '''
        self.parameters.declare('pointset', default=1.)
        self.parameters.declare('mass', default=1., types=(float, np.ndarray)) # NEED SOME KIND OF MPCOMP MARKER
        # self.parameters.declare('inertia', default=1., types=(float, np.ndarray)) # NEED SOME KIND OF MPCOMP MARKER

class BatteryFeature(PowertrainComponent):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        # self.parameters.declare('name', types=str)
        self.parameters.declare('level', default=1.0, lower=0.0, upper=1.0) # IGNORE
        self.parameters.declare('n_parallel', types=CsdlInputVariableInfo)

class ConverterFeature(PowertrainComponent):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        # self.parameters.declare('name', types=str)
        self.parameters.declare('type', default='dc-dc', values=['ac-dc', 'dc-dc'])

class DCBusFeature(PowertrainComponent):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        # self.parameters.declare('name', types=str)

class MotorFeature(PowertrainComponent):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        # self.parameters.declare('name', types=str)
        self.parameters.declare('num_rotors', types=int, default=1) # IGNORE

    def set_num_rotors(self, num_rotors=1):
        self.num_rotors=num_rotors


''' ---- POWERTRAIN BASECLASS ---- '''
class Powertrain(BaseClass):
    def initialize(self, kwargs):
        super().initialize(kwargs)
        self.parameters.declare('name', types=str)

        self.battery_nodes = []
        self.dcdc_converter_nodes = []
        self.dc_bus_nodes = []
        self.acdc_converter_nodes = []
        self.motor_nodes = []

        self.battery_names = []
        self.dcdc_converter_names = []
        self.dc_bus_names = []
        self.acdc_converter_names = []
        self.motor_names = []

        self.powertrain_connections = []

    def add_input_branch(self, dcbus, components=[]): # IGNORE FOR NOW
        for component in components:
            self.add_node(component)

        if dcbus not in self.dc_bus_nodes:
            self.add_node(dcbus)

    def add_output_branch(self, dcbus, components=[]): # IGNORE FOR NOW
        for component in components:
            self.add_node(component)

        if dcbus not in self.dc_bus_nodes:
            self.add_node(dcbus)

    def add_node(self, node=None):
        if isinstance(node, BatteryFeature):
            if node not in self.battery_nodes:
                self.battery_nodes.append(node)
                self.battery_names.append(node.parameters['name'])

        elif isinstance(node, DCBusFeature):
            if node not in self.dc_bus_nodes:
                self.dc_bus_nodes.append(node)
                self.dc_bus_names.append(node.parameters['name'])

        elif isinstance(node, MotorFeature):
            if node not in self.motor_nodes:
                self.motor_nodes.append(node)
                self.motor_names.append(node.parameters['name'])

        elif isinstance(node, ConverterFeature):
            if node not in self.dcdc_converter_nodes and node not in self.acdc_converter_nodes:
                if node.parameters['type'] == 'ac-dc':
                    self.acdc_converter_nodes.append(node)
                    self.acdc_converter_names.append(node.parameters['name'])
                elif node.parameters['type'] == 'dc-dc':
                    self.dcdc_converter_nodes.append(node)
                    self.dcdc_converter_names.append(node.parameters['name'])

    def connect_nodes(self, start=[], stop=[]):
        # ALWAYS UPSTREAM TO DOWNSTREAM
        connections = len(start)
        if connections != len(stop):
            raise RuntimeError('Mismatched sizes between start and end nodes.')

        for node in start:
            self.add_node(node)
        for node in stop:
            self.add_node(node)
        
        for i in range(connections):
            self.powertrain_connections.append([
                start[i].parameters['name'], 
                stop[i].parameters['name']
                ]
            )

    def finalize(self):
        self.num_motors = len(self.motor_nodes)
        self.num_dcdc = len(self.dcdc_converter_nodes)
        self.num_dc_bus = len(self.dc_bus_nodes)
        self.num_acdc = len(self.acdc_converter_nodes)
        self.num_batteries = len(self.battery_nodes)

        # ''' ... '''
        
        powertrain_model = SimplePowertrainModel(
        )
        return powertrain_model, connections



''' CAN IGNORE CODE BELOW, JUST USED FOR VERIFICATION AND TESTING '''
if __name__ == '__main__':

    motor_pointsets = ['motor_ps1', 'motor_ps2', 'motor_ps3', 'motor_ps4', 'motor_ps5', 
                    'motor_ps6', 'motor_ps7', 'motor_ps8', 'motor_ps9']
    battery_pointsets = ['battery_ps1', 'batteryps2']
    dcdc_conv_pointsets = ['dcdc_converter_ps1', 'dcdc_converter_ps2']
    acdc_conv_pointsets = ['acdc_converter_ps1', 'acdc_converter_ps2', 'acdc_converter_ps3', 
                        'acdc_converter_ps4', 'acdc_converter_ps5', 'acdc_converter_ps6', 
                        'acdc_converter_ps7', 'acdc_converter_ps8', 'acdc_converter_ps9']
    dcbus_pointsets = ['dcbus_ps1', 'dcbus_ps2']

    # P = Powertrain()

    B1 = BatteryFeature(
        name='battery_1',
        pointset=battery_pointsets[0]
    )

    DD1 = ConverterFeature(
        name='dcdc_1',
        pointset=dcdc_conv_pointsets[0]
    )

    AD1 = ConverterFeature(
        name='acdc_1',
        pointset=acdc_conv_pointsets[0],
        type='ac-dc'
    )

    DB1 = DCBusFeature(
        name='dcbus_1',
        pointset=dcbus_pointsets[0]
    )

    DB2 = DCBusFeature(
        name='dcbus_2',
        pointset=dcbus_pointsets[1]
    )

    # P.add_input_branch(
    #     dcbus=DB1,
    #     components=[B1,DD1]
    # )

    # P.add_input_branch(
    #     dcbus=DB1,
    #     components=[B1,DD1]
    # )


    # P.add_node(B1)
    # P.add_node(DD1)
    # P.add_node(AD1)

