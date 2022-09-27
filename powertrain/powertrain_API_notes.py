import numpy as np 

# INITIALIZE POWERTRAIN
num_parallel_input_branches = npib # some integer number of battery-DC/DC converter pairs
num_parallel_output_branches = npob # some integer number of output branches
num_motors = nm # some integer number of motors

pt = Powertrain(
    num_input_branches=num_parallel_input_branches,
    num_output_branches=num_parallel_output_branches,
    num_motors=num_motors,
)
''' 
num_input_branches: an integer number of battery-DC/DC converter pairs upstream of the
                    DC voltage bus
num_output_branches: an integer number of branches leading DIRECTLY to an AC/DC converter
                    (or inverter); each one of these branches can contain multiple motors 
                    and loads so we will allow users to specify nodes here
num_motors: number of motors directly attached to loads (assuming one rotor per motor)

2 cases to consider:
if num_output_branches == num_motors:
    no additional nodes will be created, as each motor is paired with an inverter and on 
    parallel branches
    each motor and inverter index will be matched accordingly; if the user wants to change
    this, they can do so
else:
    user must specify nodes that connect the inverter to each motor
 
example: assume num_parallel_output_branches = 5, num_motors = 9 for L+C concept
    - the vertical rotors are connected in pairs to sets of inverters, while the pusher rotor is
        on its own branch
'''
pt.add_node(inverter_ind=1, load_ind=[1,5]) 
pt.add_node(inverter_ind=2, load_ind=[2,6])
pt.add_node(inverter_ind=3, load_ind=[3,7])
pt.add_node(inverter_ind=4, load_ind=[4,8])
pt.add_node(inverter_ind=5, load_ind=[9]) # PUSHER

# values here correspond to inverter and load indices and the connection arrangement
# there needs to be a check that ensures each load is connected somewhere, and that 
# there are no duplicates

'''
FACTORS THAT STILL NEED TO BE CONSIDERED:
    - position and location of each component within the powertrain (everything upstream of the motor)
        - position can be added by using pt.position('inverter', i, [x, y, z]) or by connecting it to the 
          geometry tool and aircraft (if the component exists in the geometry)
    - the potential to have more than one motor per rotor/load
    - multiple DC bus instances; this would create completely parallel branches starting from the battery
      as opposed to a central hub where all of the upstream components meet all of the downstream components
        - not sure if this is realistic, but good to consider if it is
'''