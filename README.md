# 274_pan_flipping
Repository to hold all the MATLAB Simulation and Hardware Interface code for team 1's 2.74 Fall 2020 final project  

<img src="/Presentation_Visuals/standard_flip.gif" width="350" height="350"/> <img src="/Presentation_Visuals/hardware_flip1.gif" width="350" height="350"/> <br/>

See our [Project_Poster here](https://github.com/Daniel-GD/274_pan_flipping/blob/main/Presentation_Visuals/team1_poster.pdf).
  
## MDEB Details ##
MBED Link: https://os.mbed.com/users/suribe/code/pan_flipping/
How to update:
1. import code into compiler
2. edit code
3. right click on program folder in 'Program Workspace' and commit the code
4. right click on program folder in 'Program Workspace' and publish code

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

 ## Authors ##
 Daniel Gonzalez Diaz  
 Laura Huang  
 Uriel Magana Salgado  
 Sebastian Uribe  
