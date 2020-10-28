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

th1_0 = pi/2;
th2_0 = 0;
dth1_0=0;
dth2_0=0;
g = 9.81;

T1_0=0;
T2_0=0;
u0= [T1_0 T2_0]';

p_arm   =  [c1; c2; l1; l2; m1; m2; I1; I2; k; kappa; th1_0; th2_0; g];
z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Parameters
l=.05;
m=.005;
I=.0001;

x0=.2;
y0=.2;
th0=0;
%     g=9.81;

dx0=0;
dy0=0;
dth0=1;

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
tf=.3;
dt=.00001;

[arm, pk, tspan]=simulate_system(z0, p, u0, tf, dt);
animate_system(arm, pk, p, tspan);