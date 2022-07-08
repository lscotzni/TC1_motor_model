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
        self.parameters.declare('rated_current')

    def define(self):
        # --- DEFINING INPUTS FROM INITIALIZE & BASIC PARAMETERS --- 
        m = self.parameters['phases']
        p = self.parameters['pole_pairs']
        Z = self.parameters['num_slots']
        I_w = self.parameters['rated_current']
        a = 1 # PARALLEL BRANCHES
        q = Z/(2*m*p) # SLOTS PER POLE PER PHASE
        mu_0 = np.pi*4e-7

        # --- RATED PARAMETERS AS INPUTS FROM OPTIMIZER ---
        D_i = self.declare_variable('D_i') # inner radius of stator
        L = self.declare_variable('L') # effective length of motor
        rated_omega = 3000
        eta_0 = 0.88 # ASSUMED INITIAL EFFICIENCY; MATLAB CODE STARTS WITH 0.88
        PF = 1 # POWER FACTOR

        f_i = rated_omega*p/60
        B_air_gap_max = 0.85 # MAX VALUE OF B IN AIR GAP
        alpha_B = 0.7 # RATIO OF B_avg/B_max IN AIR GAP [0.66, 0.71]
        kwm = 1.11 # COEFFICIENT OF B-AIR GAP CURVE (NOT USED)
        kdp1 = 0.925 # COEFFICIENT OF STATOR WINDING FUNDAMENTAL COMPONENT (NOT USED)

        line_load = 26000 # CURRENT PER UNIT LENGTH; THIS MAY NEED TO BE UPDATED
        # NOTE: THIS IS THE ELECTRIC LOADING THAT ZEYU FOUND FROM A LOOKUP TABLE IN A CHINESE TEXTBOOK;

        lambda_i = 1.25
        outer_stator_radius = D_i * lambda_i
        # outer_stator_radius = self.register_output(
        #     'outer_stator_radius',
        #     outer_stator_radius
        # )
        

        # --- POLE PITCH AND OTHER PITCHES ---
        pole_pitch = np.pi*D_i/(2*p)
        tooth_pitch = np.pi*D_i/Z
        # pole_pitch = self.register_output('pole_pitch', pole_pitch)
        # tooth_pitch = self.register_output('tooth_pitch', tooth_pitch)
        

        # --- AIR GAP LENGTH ---
        air_gap_depth = 0.4*line_load*pole_pitch/(0.9e6*B_air_gap_max)
        l_ef = L + 2*air_gap_depth
        rotor_radius = D_i - 2*air_gap_depth
        D_shaft = 0.3 * rotor_radius # outer radius of shaft
        # air_gap_depth = self.register_output('air_gap_depth', air_gap_depth)
        # l_ef = self.register_output('l_ef', l_ef) # final effective length of motor
        # rotor_radius = self.register_output('rotor_radius', rotor_radius) # D2 in MATLAB code
        
        # --- WINDINGS ---
        I_kw = I_w * eta_0 * PF
        conductors_per_phase = eta_0 * np.pi * D_i * line_load \
            / (m*I_kw)
        conductors_per_slot = m*a*conductors_per_phase/Z
        turns_per_phase = conductors_per_phase/2
        # turns_per_phase = self.register_output('turns_per_phase', turns_per_phase)
        # these lines of code will cause errors compared to MATLAB code bc Zeyu
        # uses floor to round, and we cannot do that

        J = 5 # target current density
        Acu = I_w/(a*J*conductors_per_slot) * 1e-6
        # Acu = self.register_output('Acu',Acu)
        d_coil = 2 * (Acu/np.pi)**0.5

        # --- SLOT GEOMETRY ---
        kfe = 0.95 # LAMINATION COEFFICIENT
        Bt = 1.7 # FLUX DENSITY IN STATOR TOOTH

        tooth_width = tooth_pitch * B_air_gap_max / (kfe * Bt)
        # tooth_width = self.register_output('tooth_width', tooth_width) # STATOR TOOTH WIDTH

        B_ys = 1.35 # FLUX DENSITY IN STATOR YOKE
        h_ys = (pole_pitch*alpha_B*B_air_gap_max) / (2*kfe*B_ys) # HEIGHT OF YOKE IN STATOR
        # h_ys = self.register_output('height_yoke_stator', h_ys) # HEIGHT OF YOKE IN STATOR

        theta_t = 360/Z # ANGULAR SWEEP OF STATOR SLOT IN DEGREES
        theta_sso = 0.5*theta_t
        theta_ssi = 0.3*theta_sso
        b_sb = theta_ssi*np.pi*D_i/360 # WIDTH OF BOTTOM OF SLOT
        h_slot = (outer_stator_radius - D_i)/2 - h_ys # HEIGHT OF SLOT
        # b_sb = self.register_output('slot_bottom_width', b_sb) # WIDTH OF BOTTOM OF SLOT
        # h_slot = self.register_output('slot_height', h_slot) # HEIGHT OF SLOT

        h_k = 0.0008 # NOT SURE HWAT THIS IS
        h_os = 1.5 * h_k # NOT SURE WHAT THIS IS

        b_s1 = (np.pi*(D_i+2*(h_os+h_k)))/36 - tooth_width # RADIALLY INNER WIDTH OF SLOT
        # b_s1 = self.register_output('slot_width_inner', b_s1) # RADIALLY INNER WIDTH OF SLOT
        b_s2 = (np.pi*(D_i+2*h_slot))/36 - tooth_width # RADIALLY OUTER WIDTH OF SLOT

        Tau_y = np.pi*(D_i+h_slot) / (2*p)
        L_j1 = np.pi*(outer_stator_radius-h_ys) / (4*p) # STATOR YOKE LENGTH FOR MAGNETIC CIRCUIT CALCULATION
        A_slot = (b_s1+b_s2)*(h_slot-h_k-h_os)/2
        # Tau_y = self.register_output('Tau_y', Tau_y)
        # L_j1 = self.register_output('L_j1', L_j1) # STATOR YOKE LENGTH FOR MAGNETIC CIRCUIT CALCULATION

        # --- WINDING FACTOR ---
        Kp1 = csdl.sin(pole_pitch*90*np.pi/pole_pitch/180) # INTEGRAL WINDING

        alpha = 360*p/Z # ELECTRICAL ANGLE PER SLOT
        Kd1 = np.sin(q*alpha/2)/(q*np.sin(alpha/2))
        Kdp1 = Kd1*Kp1
        # Kdp1 = self.register_output('Kdp1', Kdp1)

        # --- MAGNET GEOMETRY ---
        hm = 0.004 # MAGNET THICKNESS
        theta_p = 360/2/p # ANGULAR SWEEP OF POLE IN DEGREES
        theta_m = 0.78*theta_p
        Dm = rotor_radius - 0.002
        bm = Dm*np.pi*theta_m/360
        # bm = self.register_output('bm', bm)

        T = 75 # assuming normal operating temp
        Br_20 = 1.2 # Br at 20 C
        alpha_Br = -0.12 # temperature coefficients
        IL = 0 # revercible loss
        Br = 1+(T-20)*alpha_Br/100*(1-IL/100)*Br_20

        Hc_20 = 907000; # coercivity
        Hc = (1+(T-20)*alpha_Br/100)*(1- IL/100)*Hc_20;
        mu_r = Br/mu_0/Hc; # relative permeability

        # Br = 1.2 # MAGNET REMANENCE
        # Hc = 907000. # MAGNET COERCIVITY

        mu_r = Br/(mu_0*Hc) # RELATIVE MAGNET PERMEABILITY

        Am_r = bm*l_ef # RADIAL CROSS SECTIONAL AREA OF MAGNET
        # Am_r = self.register_output('Am_r', Am_r) # RADIAL CROSS SECTIONAL AREA OF MAGNET
        rho_magnet = 7.6 # MAGNET DENSITY (g/cm^3)
        mass_magnet = 2*p*bm*hm*l_ef*rho_magnet*1e3 # MAGNET MASS

        phi_r = Br*Am_r
        # phi_r = self.register_output('phi_r',phi_r)
        Fc = 2*Hc*hm

        lambda_m = phi_r/Fc
        # lambda_m = self.register_output('lambda_m',lambda_m)
        alpha_p1 = bm/pole_pitch
        alpha_i = alpha_p1+4/((pole_pitch/air_gap_depth)+(6/(1-alpha_p1)))
        # alpha_i = self.register_output('alpha_i', alpha_i)

        Kf = 4*csdl.sin(alpha_i*np.pi/2)/np.pi # COEFF OF MAGNETIC FLUX DENSITY ALONG AIR GAP
        K_phi = 8.5*csdl.sin(alpha_i*np.pi/2)/(np.pi**2*alpha_i) # COEFF OF FLUX ALONG AIR GAP
        # Kf = self.register_output('Kf', Kf) # COEFF OF MAGNETIC FLUX DENSITY ALONG AIR GAP
        # K_phi = self.register_output('K_phi', K_phi) # COEFF OF FLUX ALONG AIR GAP
        K_theta1 = tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)/(tooth_pitch*(4.4*air_gap_depth + 0.75*b_sb)-b_sb**2)
        K_theta2 = 1 # no rotor slot

        K_theta = K_theta1*K_theta2
        # K_theta = self.register_output('K_theta', K_theta)
        l_f2 = hm
        A_f2 = l_f2*l_ef
        # A_f2 = self.register_output('A_f2', A_f2)

        # --- RESISTANCE & MASS CALCULATION
        rho = 0.0217e-6 # RESISTIVITY ------ GET CLARIFICATION ON UNITS
        l_B = l_ef + 2*0.01 # straight length of coil
        l_coil = l_B + 2.0*pole_pitch # length of half-turn
        
        Rdc = self.register_output(
            'Rdc',
            2 * rho * turns_per_phase * l_coil / \
            (a * Acu * conductors_per_slot) # DC RESISTANCE
        )

        Rdc1 = self.register_output(
            'Rdc1',
            2*rho*turns_per_phase*l_coil / \
            (a*np.pi/2*d_coil**2) # DC RESISTANCE
        )
        # ZEYU'S CODE USES Rdc1 AND Rdc IN DIFFERENT PLACES; ASK ZEYU ABOUT THIS

        delta = (rho/(np.pi*mu_0*f_i)) ** 0.5

        Rac = self.register_output(
            'Rac',
            Rdc / ((2*delta/d_coil) - (delta/d_coil)**2)
        ) # NOT ACCURATE AND INCORRECT COMPARED TO ZEYU'S CODE

        C = 1.05
        rho_cu = 8.9 # mass density of copper in g/cm^3
        mass_cu = C*l_coil*conductors_per_slot*Z*Acu*rho_cu*1e3 # mass of copper

        rho_fe = 7.8 # mass density of iron in g/cm^3
        mass_deficit_slot = A_slot*l_ef*rho_fe*Z*1e3
        mass_deficit_mag = 2*p*bm*hm*l_ef*rho_fe*1e3

        self.register_output(
            'motor_mass', 
            (mass_cu-mass_deficit_slot) + (mass_magnet-mass_deficit_mag) + \
            np.pi*l_ef*rho_fe*((outer_stator_radius/2)**2 - (D_shaft/2)**2)*1e3
        )

        # POPULATE motor_variables WITH THE register_output CALLS 
        # THE ONLY ONES THAT WILL BE IGNORED ARE:
        #   - motor_mass
        #   - resistance (Rdc, Rac)

        # rated frequency f_i is no longer a CSDL object; will need to calculate this
        # in each individual model given a rated omega

        motor_variables = self.create_output(
            'motor_variables',
            shape=(25,)
        )
        
        motor_variables[0]  = outer_stator_radius
        motor_variables[1]  = pole_pitch
        motor_variables[2]  = tooth_pitch
        motor_variables[3]  = air_gap_depth
        motor_variables[4]  = l_ef
        motor_variables[5]  = rotor_radius
        motor_variables[6]  = turns_per_phase
        motor_variables[7]  = Acu
        motor_variables[8]  = tooth_width
        motor_variables[9]  = h_ys
        motor_variables[10] = b_sb
        motor_variables[11] = h_slot
        motor_variables[12] = b_s1
        motor_variables[13] = Tau_y
        motor_variables[14] = L_j1
        motor_variables[15] = Kdp1
        motor_variables[16] = bm
        motor_variables[17] = Am_r
        motor_variables[18] = phi_r
        motor_variables[19] = lambda_m
        motor_variables[20] = alpha_i
        motor_variables[21] = Kf
        motor_variables[22] = K_phi
        motor_variables[23] = K_theta
        motor_variables[24] = A_f2