import numpy as np

from powertrain_features import MotorFeature, BatteryFeature, ConverterFeature, DCBusFeature
from powertrain_features import Powertrain
'''
POWERTRAIN TEST SCRIPT:
    - L+C SIMPLIFIED CONCEPT WITH 1 PUSHER, 2 VERTICAL ROTORS

POWERTRAIN SPECS:
- 2 BATTERIES
- 1 DC BUS
- 3 PARALLEL OUTPUT BRANCHES
    - HOVER ROTORS IN PAIRS (2) + 1 FOR PUSHER ROTOR BRANCH
- 1 MOTOR + 2 ROTORS FOR EACH PROP SYSTEM, 1 ROTOR & MOTOR FOR PUSHER
'''

# POINTSETS FOR EACH PIECE OF THE POWERTRAIN
motor_pointsets = ['motor_ps1', 'motor_ps2', 'motor_ps3']
battery_pointsets = ['battery_ps1', 'batteryps2']
dcdc_conv_pointsets = ['dcdc_converter_ps1', 'dcdc_converter_ps2']
acdc_conv_pointsets = ['acdc_converter_ps1', 'acdc_converter_ps2', 'acdc_converter_ps3']
dcbus_pointsets = ['dcbus_ps1']


# INITIALIZE POWERTRAIN OBJECT
p = Powertrain()

battery_1 = BatteryFeature(
    name='battery_1',
    pointset=battery_pointsets[0],
    level=0.4 # user-defined battery degradation
)
# other option is to use battery_1.add_pointset(...)

battery_2 = BatteryFeature(
    name='battery_2',
    pointset=battery_pointsets[1]
)

# another option:
[battery_1, battery_2] = BatteryFeature(
    base_name='battery',
    pointsets=battery_pointsets,
    level=[1.0, 0.4]
)

dcdc_1 = ConverterFeature(
    name='dcdc_1',
    pointset=dcdc_conv_pointsets[0]
)

dcdc_2 = ConverterFeature(
    name='dcdc_2',
    pointset=dcdc_conv_pointsets[1]
)

dcbus_1 = DCBusFeature(
    name='dcbus_1',
    pointset=dcbus_pointsets[0],
    output_branches=3
)

acdc_1 = ConverterFeature(
    name='acdc_1',
    pointset=acdc_conv_pointsets[0],
    type='ac-dc',
    output_branches=1
)

acdc_2 = ConverterFeature(
    name='acdc_2',
    pointset=acdc_conv_pointsets[1],
    type='ac-dc',
    efficiency=0.95
)

acdc_3 = ConverterFeature(
    name='acdc_3',
    pointset=acdc_conv_pointsets[2],
    type='ac-dc',
    efficiency=0.98
)

motor_1 = MotorFeature(
    name='motor_1',
    pointset=motor_pointsets[0],
    num_rotors=2
)

motor_2 = MotorFeature(
    name='motor_2',
    pointset=motor_pointsets[1],
    num_rotors=2
)

motor_3 = MotorFeature( # PUSHER, DEFAULT NUM_ROTORS
    name='motor_3',
    pointset=motor_pointsets[2],
)

# what might be better is to define a single feature, and then output a list based on pointsets
# ex: [a, b, c, d, e] = RandomFeature(basename='random', pointset=[psa, psb, psc, psd, pse])
''' NODE CONNECTIONS BEFORE DC BUS '''
p.add_input_branch(
    dcbus=dcbus_1,
    components=[battery_1, dcdc_1]
)

p.connect_nodes(
    start=[battery_2, dcdc_2],
    end=[dcdc_2, dcbus_1]
)

''' NODE CONNECTIONS AFTER DC BUS '''
p.add_output_branch(
    dcbus=dcbus_1,
    components=[acdc_1, motor_1]
)

p.add_output_branch(
    dcbus=dcbus_1,
    components=[acdc_2, motor_2]
)

p.connect_nodes(
    start=[dcbus_1, acdc_3],
    end=[acdc_3, motor_3]
)

p.assemble() 
# THIS STEP MAKES THE PROPER CONNECTIONS AND ADDS INFO TO CSDL MODELS
''' 
NOTES: 
    - mass has not been added to these, but I have added them as a parameter input 
        - avoided adding it until further discussion/feedback
    - two methods to connect nodes together:
        - connect_nodes, a more general approach that can connect any two components
        - add_input/output_branch, for more simplified powertrain layouts
'''