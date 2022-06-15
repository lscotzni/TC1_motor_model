import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

from TC1_magnet_mec_model import TC1MotorMECModel

class TC1MotorSizingModel(Model):
    '''
    INPUTS TO THIS MODEL:
        - tau_max (maximum torque)
        - D_rotor (rotor diameter)
        - omega_base (base speed of motor)

    OUTPUTS OF THIS MODEL:
        - L_motor (motor length)
        - D_motor (motor diameter)
    '''
    def initialize(self):
        self.parameters.declare('fit_coeff_dep_H') # FITTING COEFFICIENTS (X = H)
        self.parameters.declare('fit_form_dep_H') # FITTING EQUATION FORM (B = f(H))

        self.parameters.declare('fit_coeff_dep_B') # FITTING COEFFICIENTS (X = B)
        self.parameters.declare('fit_form_dep_B') # FITTING EQUATION FORM (H = g(B))

    def fitting_dep_H(self, H):
        # f = 0
        # order = 10
        # for i in range(order + 1):
        #     f += self.fit_coeff_dep_H[i] * H**(order-i)

        f = []
        order = 10
        for i in range(order + 1):
            f.append(self.fit_coeff_dep_H[i] * H**(order-i))
        return csdl.sum(*f)
    ''' FOR LOOP MIGHT BE A PROBLEM BECAUSE OBJECT H IS BEING APPENDED TO A NORMAL PYTHON LIST '''

    def fitting_dep_B(self, B):
        a = self.fit_coeff_dep_B[0]
        b = self.fit_coeff_dep_B[1]
        c = self.fit_coeff_dep_B[2]
        f = (a*csdl.exp(b*B + c) + 200) * B**1.4
        return f

    def define(self):
        # --- B-H CURVE FITTING COEFFICIENT INPUTS ---
        self.fit_coeff_dep_H = self.parameters['fit_coeff_dep_H']
        self.fit_coeff_dep_B = self.parameters['fit_coeff_dep_B']

        # --- INPUTS FROM PREVIOUS MODELS ---
        tau_max = self.declare_variable('tau_max') # MAX TORQUE
        omega_base = self.declare_variable('omega_base') # MOTOR BASE/RATED SPEED IN RPM
        omega = self.declare_variable('omega')
        D_rotor = self.declare_variable('D_rotor') # ROTOR DIAMETER

        # --- BASIC MOTOR PARAMETERS ---
        m = self.declare_variable('phases', val=3) # NUMBER OF PHASES
        p = self.declare_variable('pole_pairs', val=6) # POLE PAIRS
        Z = self.declare_variable('num_slots', val=36) # NUMBER OF SLOTS
        q = self.register_output('slots_per_pole_per_phase', var=Z/(2*m*p)) # SLOTS PER POLE PER PHASE
        a = self.declare_variable('parallel_branches', val=1.0)

        eta_0 = self.declare_variable('eta_0', val=0.88) # ASSUMED INITIAL EFFICIENCY; MATLAB CODE STARTS WITH 0.88

        f_i = self.register_output('frequency', var=omega_base*p/60) # FREQUENCY
        mu_0 = self.declare_variable('vacuum_permeability', np.pi*4e-7)

        # --- CALCULATING SIZE PARAMETER & MOTOR DIMENSIONS ---
        rated_power = self.declare_variable('rated_power')
        rated_rpm = self.declare_variable('rated_rpm')
        rated_phase_voltage = self.declare_variable('rated_phase_voltage')
        power_current = rated_power/(m*rated_phase_voltage)
        
        B_air_gap_max = self.declare_variable('B_air_gap_max', val=0.85) # MAX VALUE OF B IN AIR GAP
        alpha_B = self.declare_variable('alpha_B', val=0.7) # RATIO OF B_avg/B_max IN AIR GAP [0.66, 0.71]
        kwm = self.declare_variable('kwm', val=1.11) # COEFFICIENT OF B-AIR GAP CURVE
        kdp1 = self.declare_variable('kdp1', val=0.925) # COEFFICIENT OF STATOR WINDING FUNDAMENTAL COMPONENT

        line_load = self.declare_variable('line_load') # CURRENT PER UNIT LENGTH; THIS WILL NEED TO BE UPDATED
        # NOTE: THIS IS THE ELECTRIC LOADING THAT ZEYU FOUND FROM A LOOKUP TABLE IN A CHINESE TEXTBOOK;
        # IT CAN SOMEWHAT BE ASSUMED AS CONSTANT; NEED TO FURTHER LOOK INTO THIS

        P_calc = self.register_output(
            'P_calc',
            rated_power*eta_0
        )

        motor_size_parameter = self.register_output(
            'motor_size_parameter',
            (6.1 * P_calc) / (kwm * kdp1 * line_load * alpha_B * B_air_gap_max * omega_base)
        )

        length_to_pole_pitch_ratio = 2.8 # CALLED AS lambda IN ZEYU'S MATLAB CODE

        inner_stator_radius = self.register_output( # INNER STATOR diameter (D_i in MATLAB code)
            'inner_stator_radius',
            ((2*p*motor_size_parameter)/(length_to_pole_pitch_ratio*np.pi))**(1/3)
        )

        l_ef = self.register_output(
            'l_ef',
            motor_size_parameter / (inner_stator_radius)**2
        )

        stator_outer_inner_diameter_ratio = 1.25

        outer_stator_radius = self.register_output(
            'outer_stator_radius',
            inner_stator_radius * stator_outer_inner_diameter_ratio
        )

        # --- POLE PITCH AND OTHER PITCHES ---
        pole_pitch = self.register_output('pole_pitch', np.pi*inner_stator_radius/(2*p))
        tooth_pitch = self.register_output('tooth_pitch', np.pi*inner_stator_radius/Z)

        # --- AIR GAP LENGTH ---
        air_gap_depth = self.register_output('air_gap_depth', 0.4*line_load*pole_pitch/(0.9e-6*B_air_gap_max))
        rotor_radius = self.register_output('rotor_radius', inner_stator_radius - 2*air_gap_depth)

        # --- WINDINGS ---
        conductors_per_phase = eta_0 * np.pi * inner_stator_radius * line_load \
            / (m*power_current)
        conductors_per_slot = m*a*conductors_per_phase/Z
        turns_per_phase = self.register_output(
            'turns_per_phase',
            conductors_per_phase/2
        )

        J = 5 # target current density
        I_w = self.register_output('I_w',power_current/eta_0)
        Acu = self.register_output(
            'Acu',
            I_w/(a*J*conductors_per_slot) * 1e-6,
        )
        r_coil = 1 * (Acu/np.pi)**0.5

        # --- SLOT GEOMETRY ---
        kfe = self.declare_variable('lamination_coefficient', 0.95) # LAMINATION COEFFICIENT
        B_t = self.declare_variable('tooth_flux_density', 1.7) # FLUX DENSITY IN STATOR TOOTH

        tooth_width = self.register_output('tooth_width', tooth_pitch * B_air_gap_max / (kfe * B_t)) # STATOR TOOTH WIDTH

        B_ys = self.declare_variable('B_yoke_stator', val=1.35) # FLUX DENSITY IN STATOR YOKE
        h_ys = self.register_output(
            'height_yoke_stator', 
            (pole_pitch*alpha_B*B_air_gap_max) / (2*kfe*B_ys)
        ) # HEIGHT OF YOKE IN STATOR

        theta_t = self.register_output('angle_per_slot', 360/Z) # ANGULAR SWEEP OF STATOR SLOT IN DEGREES
        theta_sso = self.register_output('theta_sso', 0.5*theta_t)
        theta_ssi = self.register_output('theta_ssi', 0.3*theta_sso)

        b_sb = self.register_output('slot_bottom_width', theta_ssi*np.pi*inner_stator_radius/360) # WIDTH OF BOTTOM OF SLOT

        h_slot = self.register_output('slot_height', (outer_stator_radius - inner_stator_radius)/2 - h_ys) # HEIGHT OF SLOT

        h_k = 0.0008 # NOT SURE HWAT THIS IS
        h_os = 1.5 * h_k # NOT SURE WHAT THIS IS

        b_s1 = self.register_output('slot_width_inner', (np.pi*(inner_stator_radius+2*(h_os+h_k)))/36 - tooth_width) # RADIALLY INNER WIDTH OF SLOT
        b_s2 = self.register_output('slot_width_outer', (np.pi*(inner_stator_radius+2*h_slot))/36 - tooth_width) # RADIALLY OUTER WIDTH OF SLOT

        A_slot = self.register_output('slot_area', (b_s1+b_s2)*(h_slot-h_k-h_os)/2) # AREA OF SLOT
        Tau_y = self.register_output('Tau_y', np.pi*(inner_stator_radius+h_slot) / (2*p))
        L_j1 = self.register_output('L_j1', np.pi*(outer_stator_radius-h_ys) / (4*p)) # STATOR YOKE LENGTH FOR MAGNETIC CIRCUIT CALCULATION

        # --- WINDING FACTOR ---
        y1 = self.register_output('y1', pole_pitch)
        Kp1 = self.register_output('Kp1', csdl.sin(y1*90*np.pi/pole_pitch/180)) # INTEGRAL WINDING

        alpha = self.register_output('electric_angle_per_slot', 360*p/Z) # ELECTRICAL ANGLE PER SLOT
        Kd1 = self.register_output('Kd1', csdl.sin(q*alpha/2)/(q*csdl.sin(alpha/2)))
        Kdp1 = self.register_output('Kdp1', Kd1*Kp1)

        # --- MAGNET GEOMETRY ---
        hm = self.declare_variable('magnet_thickness', 0.004) # MAGNET THICKNESS
        theta_p = self.register_output('theta_p', 360/2/p) # ANGULAR SWEEP OF POLE IN DEGREES
        theta_m = self.register_output('theta_m', 0.78*theta_p)
        Dm = self.register_output('Dm', rotor_radius - 0.002)
        bm = self.register_output('bm', Dm*np.pi*theta_m/360)

        Br = self.declare_variable('magnet_residual_induction', 1.2) # MAGNET REMANENCE
        Hc = self.declare_variable('magnet_coercivity',907000.) # MAGNET COERCIVITY

        mu_r = self.register_output('magnet_relative_permeability', Br/(mu_0*Hc)) # RELATIVE MAGNET PERMEABILITY

        Am_r = self.register_output('magnet_area_radial_view', bm*l_ef) # RADIAL CROSS SECTIONAL AREA OF MAGNET
        rho_m = self.declare_variable('magnet_density', 7.6) # MAGNET DENSITY

        mass_m = self.register_output('magnet_mass',2*p*bm*hm*l_ef*rho_m*1e3) # MAGNET MASS

        phi_r = self.register_output('phi_r',Br*Am_r)
        Fc = self.register_output('Fc', 2*Hc*hm)

        lambda_m = self.register_output('lambda_m',phi_r/Fc)
        alpha_p1 = self.register_output('alpha_p1', bm/pole_pitch)
        alpha_i = self.register_output('alpha_i', alpha_p1+4/((pole_pitch/air_gap_depth)+(6/(1-alpha_p1))))

        Kf = self.register_output('Kf', 4*csdl.sin(alpha_i*np.pi/2)/np.pi) # COEFF OF MAGNETIC FLUX DENSITY ALONG AIR GAP
        K_phi = self.register_output('K_phi', 8*csdl.sin(alpha_i*np.pi/2)/(np.pi**2*alpha_i)) # COEFF OF FLUX ALONG AIR GAP
        K_theta1 = self.register_output('K_theta1', tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)/(tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)-b_sb**2))
        K_theta2 = self.declare_variable('K_theta2', 1) # no rotor slot

        K_theta = self.register_output('K_theta', K_theta1*K_theta2)

        l_f1 = self.declare_variable('l_f1', 0.001)
        l_f2 = self.register_output('l_f2', hm)

        A_f1 = self.register_output('A_f1', l_f1*l_ef)
        A_f2 = self.register_output('A_f2', l_f2*l_ef)
        K_sigma_air = self.declare_variable('K_sigma_air', val=1.2336) # COEFFICIENT OF LEAKAGE IN AIR GAP

        # --- RESISTANCE & MASS CALCULATION
        rho = 0.0217e-6 # RESISTIVITY ------ GET CLARIFICATION ON UNITS
        l_B = l_ef + 2*0.01 # straight length of coil
        l_coil = l_B + 1.5 * pole_pitch # length of half-turn
        
        Rdc = 2 * rho * turns_per_phase * l_coil / \
            (a * Acu * conductors_per_slot) # DC RESISTANCE

        delta = (rho/(np.pi*mu_0*f_i)) ** 0.5

        Rac = Rdc / ((2*delta/r_coil) - (delta/r_coil)**2)

        C = 1.05
        rho_cu = 8960. # mass density of copper in kg/m^3
        mass_cu = C * l_coil * conductors_per_slot**2 * Z * Acu * rho_cu # mass of copper

        rho_fe = 7874. # mass density of iron in kg/m^3
        mass_fe = kfe*l_ef*rho_fe*(outer_stator_radius*100 + 0.05)**2 * 1e-3 * 0.1 # mass of iron

        '''------------------- EVERYTHING BELOW THIS IS NOT NEEDED -----------------------'''

        # ''' MEC MODEL '''
        # self.add(
        #     'MEC_Model',
        #     TC1MotorMECModel(
        #         fit_coeff_dep_H=self.fit_coeff_dep_H,
        #         fit_coeff_dep_B=self.fit_coeff_dep_B
        #     ),
        # )
        # phi_sum = self.declare_variable('phi_sum') # SUM OF FLUX
        # F_sum = self.declare_variable('F_sum') # FROM MEC
        # F_delta = self.declare_variable('F_delta') # FROM MEC
        # lambda_n = self.declare_variable('lambda_n')
        # lambda_leak_standard = self.declare_variable('lambda_leak_standard')
        
        # E_o = 4.44*f_i*Kdp1*turns_per_phase*phi_sum*K_phi

        # # --- INDUCTANCE CALCULATIONS ---

        # # --- d-axis ---
        # K_st = F_sum/F_delta
        # Cx = (4*np.pi*f_i*mu_0*l_ef*(kdp1*turns_per_phase)**2) / p
        # lambda_U1 = (h_k/b_sb) + (2*h_os/(b_sb+b_s1));
        # lambda_L1 = 0.45
        # lambda_S1 = lambda_L1 + lambda_U1
        # X_s1 = (2*p*m*l_ef*lambda_S1*Cx) / (l_ef*Z*Kdp1**2)
        # s_total = 0.0128
        # X_d1 = (m*pole_pitch*s_total*Cx) / (air_gap_depth*K_theta*K_st*(np.pi*Kdp1)**2)
        # X_E1 = 0.47*Cx*(l_B - 0.64*Tau_y)/(l_ef*Kdp1**2)
        # X_1 = X_s1+X_d1+X_E1
        # Kad = 1/Kf
        # Kaq = 0.36/Kf

        # I_d = I_w/2
        # F_ad = 0.45*m*Kad*Kdp1*turns_per_phase*I_d/p
        # f_a = F_ad / (K_sigma_air*hm*Hc)
        # bm_N = (lambda_n*(1-f_a)) / (lambda_n + 1) # lambda_n comes from MEC
        # phi_air_N = (bm_N*(1-bm_N)*lambda_leak_standard) * Am_r * Br
        # E_d = 4.44*f_i*Kdp1*turns_per_phase*phi_air_N*K_phi # EM at d-axis
        # Xad  = ((E_o-E_d)**2)**0.5/I_d
        # Xd = Xad + X_1
        # Ld = Xd / (2*np.pi*f_i) # d-axis inductance

        # # --- q-axis ---
        # phi_aq = 0.35*phi_sum
        # B_aq  = phi_aq/(alpha_i*l_ef*pole_pitch)
        # F_sigma_q = 1.6*B_aq*(K_theta*air_gap_depth)/mu_0 # MAGNETIC STRENGTH ON Q-AXIS

        # B_t_q = B_aq*tooth_pitch/(tooth_width*kfe)
        # H_t1_q = self.fitting_dep_B(B_t_q)
        # F_t1_q = 2*H_t1_q*h_slot # DIFF OF MAGNETIC POTENTIAL ALONG TOOTH  

        # B_j1_q = phi_aq/(2*l_ef*h_ys)
        # H_j1_q = self.fitting_dep_B(B_j1_q)
        # C_1 = 0.3 # NOT USED ANYWHERE
        # F_j1_q = 2*H_j1_q*L_j1
        # F_total_q = F_sigma_q + F_t1_q + F_j1_q # TOTAL MAGNETIC STRENGTH ON Q-AXIS
        # I_q = p*F_total_q/(0.9*m*Kaq*Kdp1*turns_per_phase) #  CURRENT AT Q-AXIS
        # E_aq = phi_aq*E_o/phi_sum # EMF @ Q-AXIS
        # Xaq = E_aq/I_q
        # Xq = Xaq + X_1
        # Lq = Xq/(2*np.pi*f_i)




