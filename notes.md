# Things to mention at meetings with Darshan
data-fitted smooth models exist for materials; should I make them inputs or should
I just add the coefficients manually?
  - add as inputs to the initial model; allows for generality with other materials
ask Darshan how the RPM for each flight condition is chosen *(I believe this is a DV)*
still not totally sure about what is needed from me or how to set models up for caddee
  - what do I need to alter in my models to fit them in?
  - my model won't need any of the 12 "universal" inputs/variables, do I still need to consider those in the model I'm working on?
  - how are constraints handled here? ex: I need to incorporate the max voltage into the analysis models
Still running into issues with the sizing model and the analysis models regarding brackets

motor sizing has 28 register_outputs that feed into the 5 different submodels in analysis
  - 3 input parameters (phases, slots, poles)

=========================================================================================
# Things to mention at meeting with Zeyu/Shuofeng/Gabe / NOTES FROM MOTOR MEETING
confirm the state phi_aq for inductance MEC and the bracket
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
models to work on:
  - flux weakening (do analytical method)

do a full verification of the motor models
  - sizing: *DONE*
  - analysis:
    - magnet MEC: *DONE*
    - inductance MEC: *DONE*
    - flux weakening: need to verify result
    - MTPA: need to verify result
    - post-processing: need to verify

fix permeability fitting for B = f(H) *DONE* and write as a class

mass-torque fitting: get coefficients and plug into CSDL model

FIX phi_m CALCULATION ACCORDING TO MATLAB CODE

POWERTRAIN MODEL:
- ask about the input phase current and how it's different from Id and Iq in the motor model
- calculate the voltage components using the MATLAB code as reference
- how much freedom is given to the user?
  - branches for battery & DC/DC converter before the bus
  - is there going to be an option with multiple motors on the rotors?