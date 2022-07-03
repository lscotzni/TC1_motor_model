import numpy as np
import csdl
from csdl_om import Simulator

from TC1_motor_model.analysis_models.TC1_inductance_mec_model import InductanceModel
from TC1_motor_model.permeability.mu_fitting import permeability_fitting

# SETUP PERMEABILITY FITTING
file_name = 'TC1_motor_model/permeability/Magnetic alloy, silicon core iron C.tab'
order=10

mu_fitting = permeability_fitting(
    file_name=file_name,
    test=False,
    order=order
)

fit_coeff_dep_H = mu_fitting[0]
fit_coeff_dep_B = mu_fitting[1]

# CSDL MODEL RUN
m = InductanceModel(
    pole_pairs=6,
    phases=3,
    num_slots=36,
    fit_coeff_dep_H=fit_coeff_dep_H,
    fit_coeff_dep_B=fit_coeff_dep_B,
)

sim = Simulator(m)
# EXISTS IN THE INDUCTANCE MODEL
sim['phi_air'] = 0.0151
sim['F_total'] = 1.4415e+03
sim['F_delta'] = 1.3583e+03
sim['f_i'] = 300
sim['l_ef'] = 0.2755
sim['turns_per_phase'] = 36 # W_1 in matlab code
sim['slot_bottom_width'] = 0.0049
sim['slot_width_inner'] = 0.0157
sim['Kdp1'] = 1
sim['pole_pitch'] = 0.0975
sim['air_gap_depth'] = 0.0013
sim['K_theta'] = 1.0835
sim['Tau_y'] = .1037
sim['Kf'] = 1.2172
sim['I_w'] = 123.1061
sim['K_sigma_air'] = 1.2
sim['lambda_n'] = 3.6789
sim['lambda_leak_standard'] = 0.6131
sim['Am_r'] = 0.0207
sim['K_phi'] = 1.0159

# EXISTS IN THE INDUCTANCE MODEL AND NOT ABOVE
sim['alpha_i'] = 0.8104
sim['tooth_pitch'] = 0.0325
sim['tooth_width'] = 0.0171
sim['slot_height'] = 0.0239
sim['height_yoke_stator'] = 0.0226
# sim['Kaq'] = 0.2958
sim['L_j1'] = 0.058
# sim['I_d_temp'] = 72.3581


print(sim['phi_aq'])
sim.run()
# ONES NEEDED IN FUTURE COMPUTATIONS
print('phi_aq: ', sim['phi_aq'])
print('L_d: ', sim['L_d'])
print('L_q: ', sim['L_q'])
print('I_q_temp: ', sim['I_q_temp'])
print('K_st: ', sim['K_st'])
print('Cx: ', sim['Cx'])
print('lambda_U1: ', sim['lambda_U1'])
print('lambda_S1: ', sim['lambda_S1'])
print('X_s1: ', sim['X_s1'])
print('X_d1: ', sim['X_d1'])
print('X_E1: ', sim['X_E1'])
print('X_1: ', sim['X_1'])
print('f_a: ', sim['f_a'])
print('E_o: ', sim['E_o'])
print('bm_N: ', sim['bm_N'])
print('phi_air_N_temp: ', sim['phi_air_N_temp'])
print('phi_air_N: ', sim['phi_air_N'])
print('E_d: ', sim['E_d'])
print('Xad: ', sim['Xad'])
print('Xd: ', sim['Xd'])
print('aa: ', sim['aa'])
print('bb: ', sim['bb'])
print('cc: ', sim['cc'])