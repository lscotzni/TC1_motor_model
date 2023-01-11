# Things to mention at meetings with Darshan
data-fitted smooth models exist for materials; should I make them inputs or should
I just add the coefficients manually?
  - add as inputs to the initial model; allows for generality with other materials
=========================================================================================
# Things to mention at meeting with Zeyu/Shuofeng/Gabe / NOTES FROM MOTOR MEETING
Important thing to consider: is phi_aq GUARANTEED to fall between 0 and phi_air?
  it's done this way in the MATLAB code so I want to clarify this
  - bracket should be between 0 and phi_air **ALWAYS**
the MTPA and flux weakening methods use phi_air, but I thought the phi in the 
    equations should be the magnet flux (it's denoted with _m)
<< psi_m is the air gap flux, NOT the magnet >>

=========================================================================================
# CADDEE NOTES
CADDEE instantiation with some name 
feature of aircraft (like rotor, wing, etc.)
idea in future is to automate parameters of feature from geometry side
mission segments 
solver defined in mission segment must contain a feature
solver is a pure python class, but it includes an internal attribute that is a CSDL model
standard set of inputs (12 states + unique inputs that are needed for motor)
WE WILL DEFINE THE RATED SPEED AS ONE OF THESE INPUTS AS A DESIGN VARIABLE 
  also potentially include the material model
add AllowableOperatingconditions as needed

=========================================================================================
# TO-DO:

fix permeability fitting for B = f(H) *DONE* and write as a class

POWERTRAIN MODEL:
- ask about the input phase current and how it's different from Id and Iq in the motor model
  - USE THE CURRENT COMPONENTS FOR POST-PROCESSING CALCULATIONS
- use sub-dictionaries to store the information for different nodes
  - keys include in & out connections, efficiency (except battery and motor), and component-specific parameters like gear ratio

# DISCUSS WITH ZEYU + SHUOFENG:
- concerns with exact values in the efficiency map:
  - potentially add the factor "p" when converting to rad/sec (wait for Zeyu's response)

- fix the input for speed/frequency for the internal analysis models
  - I think the "omega" that is currently being used should be the **electrical** frequency, not mechanical (confirm with Zeyu later)
  - fix this during the rewrite and do so alongside Zeyu's MATLAB code