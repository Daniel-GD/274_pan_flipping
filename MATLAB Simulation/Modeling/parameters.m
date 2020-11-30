function p = parameters() 
% Arm Parameters
l1 = .084125; %Length of first link
l2= .177107; %Length of second link + pan 
c1= 0.067733; %CoM of first link
c2=0.075011; %CoM of second link + pan
pA=0.094825; %length from link2 to edge of pan
m1 = 0.273032; %Mass of first link (including motor)
m2 = 0.102036; %Mass of second link +pan
I1= 2.076e-04; %Inertia of first link
I2 = 4.040e-04; %Inertia of second link + pan
Ir1=0; %Rotor inertia of first motor
Ir2=0; %Rotor inertia of second motor
N1=0;
N2=0;

g=9.81;

k = 3;
kappa = .5;

p_arm   =  [c1; c2; l1; l2; m1; m2; I1; I2; Ir1; Ir2; N1; N2; k; kappa; g; pA];

% Pancake Parameters (quarter)
l=.02426; 
m=.00567;
thickness=.00175;
I=1/4*m*(l/2)^2+1/12*m*thickness^2;
g=9.81;

%cardboard
l=.04; %its actually .075
thickness=.005;
rho=689; % [kg/m^3]

V=pi*(l/2)^2*thickness;
m=rho*V;
I=1/4*m*(l/2)^2+1/12*m*thickness^2;
g=9.81;

p_pk=[l; m; I; g];

p= struct('arm',p_arm,'pk',p_pk);
end