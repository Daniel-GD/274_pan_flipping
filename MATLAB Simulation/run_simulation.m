%% Runs entire simulation
setpath
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = 0; %+.1;
th2_0 = pi/2;%-pi/6;
dth1_0=0; %pi/2;
dth2_0=0 ;%2*pi;%2*pi;

z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Initial Conditions
%Start the pancake at the end of the arm
pan_position= get_pan_position(z0_arm,p.arm); %Forward Kinematics of initial arm configuration
x0=pan_position(1,3)+0.03; %place the pancake at the center of mass
y0=pan_position(2,3);
th0=pi/2+th1_0+th2_0;
dx0=0; dy0=0; dth0=0;

z0_pk=[x0; y0; th0; dx0; dy0; dth0];

z0= struct('arm',z0_arm,'pk',z0_pk);

%% Define Simulation Parameters
tf=.5;
dt=.00001;
% ub=[.5 .2 -1 -1];
T1=[.5 .2 -1 -1];
T2=[.5 .4 -1 -1];
T1=[.5 .5 -.5 -.5];
T2=[-.1 .73 .5 -1];
ctrl.T1=T1; ctrl.T2=T2; ctrl.tf=tf/2;

x=[tf ctrl.tf T1 T2];
x=[0.4993    0.5890    0.7787    0.4168   -0.6808    0.3996    0.4231   -0.6759];
tf=x(1);
ctrl.tf=x(2);
ctrl.T1=x(3:5); %BUG HERE
ctrl.T2=x(6:end);



%% Run and animate simulation
[arm, pk, contact_pts, tspan]=simulate_system(z0, p, ctrl, tf, dt);
animate_system(arm, pk, contact_pts, p, tspan);