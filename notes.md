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
add the analytical brackets 
  - inductance MEC model *(DONE, NEEDS CHECKING)*
  - flux weakening *(DONE, BUT METHOD MAY CHANGE SO REVISIT)*
add the models into the proper places based on notes from meeting with Darshan *DONE*
  - need to look at video to get a reference for how to set up models in CADDEE
talk with Victor about the implicit operation
  - the bracketing isn't working (even without the variables as brackets); results fall outside of region
  - variables within brackets don't work either
  - also mention to Victor the problem with the negative signs in front of the CSDL variables

verify the other implicit models
  - still need to do MTPA and flux weakening
VERIFY UNITS ON MASS CALCULATIONS (SEEM UNREALISTIC COMPARED TO DATA SHEETS) *DONE*
  - Zeyu said they are correct and are in kg; look at new MATLAB code
make sure that all analysis models have the proper info feeding to them via variables (like f_i)

fix permeability fitting for B = f(H) *DONE* and write as a class
make sure all connections and variables in the analysis models make sense

DO ON TUESDAY 
fix sizing for new methodology *NEED TO REVIEW REST OF CODE*
  - now adjust the rest of the analysis model to reflect the changes (like not having I_w, etc.)
  - 2 outputs from motor sizing: 
    - vector with all the internal parameters called in analysis
    - motor mass
Add the torque vs mass fitting model
  - need to wait for Zeyu to gather further data
  - we will put the torque vs mass fitting into sizing
  - dynamic torque limit will be part of the analysis model

## TO DO ASAP
  - TEST AND ADJUST THE ANALYSIS MODEL TO MAKE SURE INFORMATION IS BEING PROPAGATED PROPERLY
  - CHECK OUTPUTS FROM SIZING GOING INTO ANALYSIS
  - MESSAGE VICTOR ABOUT 2 THINGS
    - HOW TO DEAL WITH MORE COMPLEX IMPLICIT METHODS
      - DECLARING VARIABLE REMOVES IT AS AN OUTPUT VARIABLE SO IT CAN'T BE EXPOSED IN THE IMPLICIT METHOD

- add max torque model to sizing
  - need to revisit fitting, but set up the model in sizing just so that coefficeints are the last thing needed
  - push this update to the github and let Darshan know
- fix variables in implicit model for T_em