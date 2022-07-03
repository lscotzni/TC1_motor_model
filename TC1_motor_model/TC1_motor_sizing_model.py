import numpy as np 
import matplotlib.pyplot as plt
from csdl_om import Simulator
from csdl import Model
import csdl

class TC1MotorSizingModel(Model):
    '''
    INPUTS TO THIS MODEL:
        - rated power
        - rated omega (rated speed of motor)

    OUTPUTS OF THIS MODEL:
        - motor geometry
        - motor mass
    '''
    def initialize(self):
        # MOTOR DISCRETE PARAMETERS
        self.parameters.declare('pole_pairs') # 6
        self.parameters.declare('phases') # 3
        self.parameters.declare('num_slots') # 36

    def define(self):
        # --- DEFINING INPUTS FROM INITIALIZE & BASIC PARAMETERS --- 
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        a = 1 # PARALLEL BRANCHES
        q = Z/(2*m*p) # SLOTS PER POLE PER PHASE
        mu_0 = np.pi*4e-7

        # --- RATED PARAMETERS AS INPUTS FROM OPTIMIZER ---
        rated_omega = self.declare_variable('rated_omega') # MOTOR BASE/RATED SPEED IN RPM
        rated_power = self.declare_variable('rated_power')
        rated_phase_voltage = 200 # EITHER 200 OR 300 ACCORDING TO ZEYU
        power_current = rated_power/(m*rated_phase_voltage)

        eta_0 = 0.88 # ASSUMED INITIAL EFFICIENCY; MATLAB CODE STARTS WITH 0.88
        f_i = self.register_output('f_i', rated_omega*p/60) # RATED FREQUENCY
        B_air_gap_max = 0.85 # MAX VALUE OF B IN AIR GAP
        alpha_B = 0.7 # RATIO OF B_avg/B_max IN AIR GAP [0.66, 0.71]
        kwm = 1.11 # COEFFICIENT OF B-AIR GAP CURVE
        kdp1 = 0.925 # COEFFICIENT OF STATOR WINDING FUNDAMENTAL COMPONENT

        line_load = 26000 # CURRENT PER UNIT LENGTH; THIS MAY NEED TO BE UPDATED
        # NOTE: THIS IS THE ELECTRIC LOADING THAT ZEYU FOUND FROM A LOOKUP TABLE IN A CHINESE TEXTBOOK;
        # IT CAN SOMEWHAT BE ASSUMED AS CONSTANT; NEED TO FURTHER LOOK INTO THIS

        P_calc = rated_power/eta_0 # EFFECTIVE POWER IN WITH ASSUMED INITIAL EFFICIENCY

        motor_size_parameter = (6.1*P_calc)/(kwm*kdp1*line_load*alpha_B*B_air_gap_max*rated_omega)
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
        turns_per_phase = self.register_output('turns_per_phase', conductors_per_phase/2)

        J = 5 # target current density
        I_w = self.register_output('I_w',power_current/eta_0)
        Acu = self.register_output(
            'Acu',
            I_w/(a*J*conductors_per_slot) * 1e-6,
        )
        r_coil = 1 * (Acu/np.pi)**0.5

        # --- SLOT GEOMETRY ---
        kfe = 0.95 # LAMINATION COEFFICIENT
        B_t = 1.7 # FLUX DENSITY IN STATOR TOOTH

        tooth_width = self.register_output('tooth_width', tooth_pitch * B_air_gap_max / (kfe * B_t)) # STATOR TOOTH WIDTH

        B_ys = 1.35 # FLUX DENSITY IN STATOR YOKE
        h_ys = self.register_output(
            'height_yoke_stator', 
            (pole_pitch*alpha_B*B_air_gap_max) / (2*kfe*B_ys)
        ) # HEIGHT OF YOKE IN STATOR

        theta_t = 360/Z # ANGULAR SWEEP OF STATOR SLOT IN DEGREES
        theta_sso = 0.5*theta_t
        theta_ssi = 0.3*theta_sso
        b_sb = self.register_output('slot_bottom_width', theta_ssi*np.pi*inner_stator_radius/360) # WIDTH OF BOTTOM OF SLOT
        h_slot = self.register_output('slot_height', (outer_stator_radius - inner_stator_radius)/2 - h_ys) # HEIGHT OF SLOT

        h_k = 0.0008 # NOT SURE HWAT THIS IS
        h_os = 1.5 * h_k # NOT SURE WHAT THIS IS

        b_s1 = self.register_output('slot_width_inner', (np.pi*(inner_stator_radius+2*(h_os+h_k)))/36 - tooth_width) # RADIALLY INNER WIDTH OF SLOT
        b_s2 = (np.pi*(inner_stator_radius+2*h_slot))/36 - tooth_width # RADIALLY OUTER WIDTH OF SLOT

        Tau_y = self.register_output('Tau_y', np.pi*(inner_stator_radius+h_slot) / (2*p))
        L_j1 = self.register_output('L_j1', np.pi*(outer_stator_radius-h_ys) / (4*p)) # STATOR YOKE LENGTH FOR MAGNETIC CIRCUIT CALCULATION

        # --- WINDING FACTOR ---
        y1 = pole_pitch
        Kp1 = csdl.sin(y1*90*np.pi/pole_pitch/180) # INTEGRAL WINDING

        alpha = 360*p/Z # ELECTRICAL ANGLE PER SLOT
        Kd1 = np.sin(q*alpha/2)/(q*np.sin(alpha/2))
        Kdp1 = self.register_output('Kdp1', Kd1*Kp1)

        # --- MAGNET GEOMETRY ---
        hm = 0.004 # MAGNET THICKNESS
        theta_p = 360/2/p # ANGULAR SWEEP OF POLE IN DEGREES
        theta_m = 0.78*theta_p
        Dm = rotor_radius - 0.002
        bm = self.register_output('bm', Dm*np.pi*theta_m/360)

        Br = 1.2 # MAGNET REMANENCE
        Hc = 907000. # MAGNET COERCIVITY

        mu_r = Br/(mu_0*Hc) # RELATIVE MAGNET PERMEABILITY

        Am_r = self.register_output('Am_r', bm*l_ef) # RADIAL CROSS SECTIONAL AREA OF MAGNET
        rho_m = 7.6 # MAGNET DENSITY (g/cm^3)
        mass_m = 2*p*bm*hm*l_ef*rho_m*1e3 # MAGNET MASS

        phi_r = self.register_output('phi_r',Br*Am_r)
        Fc = 2*Hc*hm

        lambda_m = self.register_output('lambda_m',phi_r/Fc)
        alpha_p1 = bm/pole_pitch
        alpha_i = self.register_output('alpha_i', alpha_p1+4/((pole_pitch/air_gap_depth)+(6/(1-alpha_p1))))

        Kf = self.register_output('Kf', 4*csdl.sin(alpha_i*np.pi/2)/np.pi) # COEFF OF MAGNETIC FLUX DENSITY ALONG AIR GAP
        K_phi = self.register_output('K_phi', 8.5*csdl.sin(alpha_i*np.pi/2)/(np.pi**2*alpha_i)) # COEFF OF FLUX ALONG AIR GAP
        K_theta1 = tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)/(tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)-b_sb**2)
        K_theta2 = 1 # no rotor slot

        K_theta = self.register_output('K_theta', K_theta1*K_theta2)

        l_f2 = hm
        A_f2 = self.register_output('A_f2', l_f2*l_ef)

        # --- RESISTANCE & MASS CALCULATION
        rho = 0.0217e-6 # RESISTIVITY ------ GET CLARIFICATION ON UNITS
        l_B = l_ef + 2*0.01 # straight length of coil
        l_coil = l_B + 1.5 * pole_pitch # length of half-turn
        
        Rdc = self.register_output(
            'Rdc',
            2 * rho * turns_per_phase * l_coil / \
            (a * Acu * conductors_per_slot) # DC RESISTANCE
        )

        delta = (rho/(np.pi*mu_0*f_i)) ** 0.5

        Rac = Rdc / ((2*delta/r_coil) - (delta/r_coil)**2)

        C = 1.05
        rho_cu = 8960. # mass density of copper in kg/m^3
        mass_cu = C*l_coil*conductors_per_slot**2*Z*Acu*rho_cu # mass of copper

        rho_fe = 7874. # mass density of iron in kg/m^3
        mass_fe = kfe*l_ef*rho_fe*(outer_stator_radius*100+0.05)**2*1e-3*0.1 # mass of iron

        self.register_output('motor_mass', mass_fe+mass_cu+mass_m)