# 274_pan_flipping
Repository to hold all the MATLAB code for team 1's 2.74 final project  
MBED Link: https://os.mbed.com/users/suribe/code/pan_flipping/

## How to Use: ##
Need to run setpath before running any file

## Structure: ##
### Matlab simulation ###
* Modeling -> Derives equation of motion for arm and pancake model
* AutoDerived -> Stores all the auto-generated functions from modeling
* Simulation -> Hybrid simulation of arm and pancake system
* Visualization -> Visualizes and animates results from simulation
* Misc -> Random calculations and tests
* run_simulation -> Parameters are defined here. Running this file will run the hybrid simulation and animate it

## Still need to implement: ##
- [ ] Contact Modeling between pan and pancake. (Implement simulate_contact.m)
- [ ] Add 3rd degree of freedom to arm 
- [ ] Add spring to arm
- [ ] Create optimzation code template
- [ ] MBED Side

 ## Authors ##
 Daniel Gonzalez Diaz  
 Laura Huang  
 Uriel Magana Salgado  
 Sebastian Uribe  
