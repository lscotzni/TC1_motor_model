import numpy as np
import csdl
from csdl_om import Simulator

from TC1_magnet_mec_model import MagnetMECModel
from mu_fitting import permeability_fitting

# SETUP PERMEABILITY FITTING
file_name = 'Magnetic alloy, silicon core iron C.tab'
order=10

mu_fitting = permeability_fitting(
    file_name=file_name,
    test=False,
    order=order
)

fit_coeff_dep_H = mu_fitting[0]
fit_coeff_dep_B = mu_fitting[1]

# CSDL MODEL RUN
m = MagnetMECModel(
    fit_coeff_dep_H=fit_coeff_dep_H,
    fit_coeff_dep_B=fit_coeff_dep_B,
    num_nodes=1, # NOT USED
    op_voltage=1 # NOT USED 
)

sim = Simulator(m)
# MANUALLY UPDATE VARIABLES BASED ON ZEYU'S MATLAB CODE
sim['tooth_pitch'] = 0.0244
sim['tooth_width'] = 0.0128
sim['slot_height'] = 0.0180
sim['alpha_i'] = 0.8091
sim['pole_pitch'] = .0732
sim['l_ef'] = .2069
sim['height_yoke_stator'] = 0.0170
sim['L_j1'] = 0.0435
sim['air_gap_depth'] = 9.9469e-04
sim['K_theta'] = 1.0835
sim['magnet_thickness'] = 0.0040
sim['A_f2'] = 8.2743e-04
sim['bm'] = 0.0563
sim['phi_r'] = 0.0130
sim['lambda_m'] = 1.9245e-06

print(sim['B_delta'])
sim.run()
print('B_delta: ', sim['B_delta'])
print('phi_air: ', sim['phi_air'])
# print('phi_mag: ', sim['phi_mag'])
# print('tooth_pitch: ', sim['tooth_pitch'])
# print('H_y: ', sim['H_y'])
