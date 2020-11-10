function p = parameters() 
% Arm Parameters
l1 = .084125; %Length of first link
l2= l1; %Length of second link + pan 
c1= .9*l1; %CoM of first link
c2=.5*l2; %CoM of second link + pan
m1 = .8; %Mass of first link (including motor)
m2 = .4; %Mass of second link +pan
I1=.0005; %Inertia of first link
I2 = I1; %Inertia of second link + pan
Ir1=0; %Rotor inertia of first motor
Ir2=0; %Rotor inertia of second motor

g=9.81;

k = 3;
kappa = .5;

p_arm   =  [c1; c2; l1; l2; m1; m2; I1; I2; k; kappa; g];

% Pancake Parameters (quarter)
l=.02426; 
m=.00567;
thickness=.00175;
I=1/4*m*(l/2)^2+1/12*m*thickness^2;
g=9.81;

p_pk=[l; m; I; g];

p= struct('arm',p_arm,'pk',p_pk);
end