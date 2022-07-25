import numpy as np 
from csdl import Model, GraphRepresentation
import csdl
from csdl_om import Simulator

'''
SIMPLE POWERTRAIN MODEL
- ONE BATTERY (AND ONE DC-DC CONVERTER)
- ONE DC BUS
- CHOICE OF ROTOR NUMBERS
    - EACH ON ITS OWN BRANCH, ALONG WITH 1 MOTOR AND 1 INVERTER
'''

class PowertrainOutputBranchModel(Model):
    def initialize(self):
        self.parameters.declare('instance')
        self.parameters.declare('num_nodes')

    def define(self):
        instance = self.parameters['instance']
        num_nodes = self.parameters['num_nodes']
        DCBus_voltage = 800

        input_motor_power = self.declare_variable(
            'input_motor_power_{}'.format(instance),
            shape=(num_nodes,)
        )

        input_motor_current = self.declare_variable(
            'input_motor_current_{}'.format(instance),
            shape=(num_nodes,)
        ) # IGNORE

        input_motor_voltage = self.declare_variable(
            'input_motor_voltage{}'.format(instance),
            shape=(num_nodes,)
        ) # IGNORE

        ''' === AC-DC CONVERTER === '''
        acdc_efficiency = 0.95
        acdc_input_power = self.register_output(
            'acdc_input_power_{}'.format(instance),
            input_motor_power/acdc_efficiency
        )

        acdc_input_current = self.register_output(
            'acdc_input_current_{}'.format(instance),
            acdc_input_power/DCBus_voltage
        )

class SimplePowertrainModel(Model):
    def initialize(self):
        self.parameters.declare('num_nodes')
        self.parameters.declare('num_branches')

    def define(self):
        num_nodes = self.parameters['num_nodes']
        num_branches = self.parameters['num_branches']

        parallel_branch_power = self.create_output(
            'parallel_branch_power',
            shape = (num_branches, num_nodes)
        )

        parallel_branch_current = self.create_output(
            'parallel_branch_current',
            shape = (num_branches, num_nodes)
        )

        for i in range(num_branches):
            self.add(
                PowertrainOutputBranchModel(instance=i+1, num_nodes=num_nodes),
                'powertrain_output_branch_model_{}'.format(i+1)
            )

            temp_power = self.declare_variable('acdc_input_power_{}'.format(i+1), shape=(num_nodes,))
            temp_current = self.declare_variable('acdc_input_current_{}'.format(i+1), shape=(num_nodes,))

            parallel_branch_power[i,:] = csdl.reshape(temp_power, (1, num_nodes))
            parallel_branch_current[i,:] = csdl.reshape(temp_current, (1, num_nodes))

        dcdc_converter_output_power = self.register_output(
            'dcdc_converter_output_power',
            csdl.sum(parallel_branch_power, axes=(0,))
        )

        dcdc_converter_output_current = self.register_output(
            'dcdc_converter_output_current',
            csdl.sum(parallel_branch_current, axes=(0,))
        )
        
        
        ''' === DC-DC CONVERTER === '''
        dcdc_efficiency = 0.95
        battery_output_power = self.register_output(
            'battery_output_power', 
            dcdc_converter_output_power/dcdc_efficiency
        )

        # battery_output_current = self.register_output(
        #     'battery_output_power', 
        #     dcdc_converter_output_power/dcdc_efficiency
        # )

        '''
        NOTES FOR DARSHAN:
        - output of this model for ECM: battery_output_power of shape (num_nodes,)
        - inputs of this powertrain model
            - 'input_motor_power_n' for n = 1:number of motors
                - shape is (num_nodes,)
        - MAIN POWERTRAIN FEATURES NEEDED IN SIMPLE POWERTRAIN: 
            - BatteryFeature and MotorFeature
        '''

if __name__ == '__main__':
    m = SimplePowertrainModel(
        num_nodes=3,
        num_branches=5
    )

    rep = GraphRepresentation(m)
    sim = Simulator(rep)

    sim.visualize_implementation()
    print(sim['parallel_branch_power'])
    print(sim['dcdc_converter_output_power'])