%% Runs entire simulation
setpath
%% Set parameters
% Arm Parameters
l1 = .1;
l2= .2;
c1= .9*l1;
c2=.5*l2;
m1 = .8;
m2 = .4;
I1=.005;
I2 = I1;

k = 3;
kappa = .5;

th1_0 = pi/2; %+.1;
th2_0 = -pi/6;%-pi/6;
dth1_0=pi/2;
dth2_0=2*pi;%2*pi;
g = 0;%9.81;

T1_0=0;%1;
T2_0=0;%2;

u0= [T1_0 T2_0]';

p_arm   =  [c1; c2; l1; l2; m1; m2; I1; I2; k; kappa; th1_0; th2_0; g];
z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Parameters
l=.05;
m=.005;
I=.0001;

%Start the pancake at the end of the arm
pan_position= get_pan_position(z0_arm,p_arm); %Forward Kinematics of initial arm configuration
x0=pan_position(1,3); %place the pancake at the center of mass
y0=pan_position(2,3);
th0=th2_0;

% x0=.2;%.31;
% y0=.2;%.2;
% th0=0;%-pi/4;%pi/4; %pi;

g=9.81;

dx0=0;
dy0=0;
dth0=0;

% Fx=0;
% Fy=0;
% Tau=0;
% 
% u=[Fx; Fy; Tau];

p_pk=[l; m; I; x0; y0; th0; g];
z0_pk=[x0; y0; th0; dx0; dy0; dth0];

p= struct('arm',p_arm,'pk',p_pk);
z0= struct('arm',z0_arm,'pk',z0_pk);

% Define Simulation Paramters
tf=.5;
dt=.00001;

[arm, pk, contact_pts, tspan]=simulate_system(z0, p, u0, tf, dt);
animate_system(arm, pk, contact_pts, p, tspan);