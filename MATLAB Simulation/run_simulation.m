%% Runs entire simulation
setpath
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/4;
th2_0 = -th1_0+pi/2;
dth1_0=0;
dth2_0=0;

z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Initial Conditions
%Start the pancake at the end of the arm
pan_position= get_pan_position(z0_arm,p.arm); %Forward Kinematics of initial arm configuration
x0=pan_position(1,3); %place the pancake at the center of mass of the pan
y0=pan_position(2,3);
th0=pi/2+th1_0+th2_0;
dx0=0; dy0=0; dth0=0;

z0_pk=[x0; y0; th0; dx0; dy0; dth0];

z0= struct('arm',z0_arm,'pk',z0_pk);

%% Define Simulation Parameters
tf=.5;
dt=.00001;
bezier_pts=3;

ctrl.T1=[0.3440    0.3737   -0.4953]; %pi/4 optimization
ctrl.T2 = [0.0555    0.39   -0.4141];
x=[ctrl.T1 ctrl.T2];
ctrl.T1=x(1:bezier_pts); 
ctrl.T2=x(1+bezier_pts:end);   

%% Run and animate simulation
[arm, pk, contact_pts, tspan, uout, energy]=simulate_system(z0, p, ctrl, tf, dt);
animate_system(arm, pk, contact_pts, p, tspan);