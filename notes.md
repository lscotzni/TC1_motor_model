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

Tem_1 FROM ZEYU'S CODE IS THE MAX VALUE OF THE EFFICIENCY MAP

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
change the MTPA model to just the analytical solution *(DONE)*
  - bracketing method requires solving the analytical method to set up the upper bracket
  - at that point, it's simpler to do the analytical method
test the implicit operation models to check the variable bracket method
  - REQUIRES AN UPDATE OF CSDL, CHECK VICTOR'S LAST MESSAGE
set up the pip install method properly *(WORKS FOR NOW, GET DARSHAN'S FEEDBACK)*
add the models into the proper places based on notes from meeting with Darshan
  - need to look at video to get a reference for how to set up models in CADDEE
talk with Victor about the implicit operation
  - the bracketing isn't working (even without the variables as brackets); results fall outside of region
  - variables within brackets don't work either
check the MEC model for magnet and the permeability fittings
  - lack of convergence is strange

verify the other implicit models
VERIFY UNITS ON MASS CALCULATIONS (SEEM UNREALISTIC COMPARED TO DATA SHEETS)
make pole pairs, slots and phases as input parameters for analysis models *DONE*
make sure that all analysis models have the proper info feeding to them via variables (like f_i)