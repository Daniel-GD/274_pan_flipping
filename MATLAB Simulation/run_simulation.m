%% Runs entire simulation
setpath
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/4; %+.1;
th2_0 = pi/4;%-pi/6;
dth1_0=0; %pi/2;
dth2_0=0 ;%2*pi;%2*pi;

z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Initial Conditions
%Start the pancake at the end of the arm
pan_position= get_pan_position(z0_arm,p.arm); %Forward Kinematics of initial arm configuration
x0=pan_position(1,3); %place the pancake at the center of mass
y0=pan_position(2,3);
th0=pi/2+th1_0+th2_0;
dx0=0; dy0=0; dth0=0;

z0_pk=[x0; y0; th0; dx0; dy0; dth0];

z0= struct('arm',z0_arm,'pk',z0_pk);

%% Define Simulation Parameters
tf=.5;
dt=.00001;
% ub=[.5 .2 -1 -1];
% T1=[.5 .2 -1 -1];
% T2=[.5 .4 -1 -1];
T1 = [.6 .6 -0.5 -0.5];
T1 = [.6 .6 -1 -1];
T2 = [-.1 .73 .5 -1];
% T1=[.6 .5 -.5 -.5];
% T2=[-.1 .765 .5 -1];
% T1 = [.65 .4 -.5 -.5];
% T2 = [-.1 .6 .55 -.5];
ctrl.T1=T1; ctrl.T2=T2; %ctrl.tf=tf/2;

% x=[tf ctrl.tf T1 T2];
% x= [0.5014    0.2798    0.4891    0.4919   -0.4766   -0.4811   -0.0437    0.6454    0.4135   -0.9890];
% x=[0.4993    0.5890    0.7787    0.4168   -0.6808    0.3996    0.4231   -0.6759];
% tf=x(1);
% ctrl.tf=x(2);
% ctrl.T1=x(3:5); %BUG HERE
% ctrl.T2=x(6:end);
% bezier_pts=4;
bezier_pts=3;
% x = [T1 T2];
ctrl.tf=.25;
% ctrl.T1=[.5 .5 -.5 -.5];

% ctrl.T1=x(1:1+bezier_pts-1); %BUG HERE
% ctrl.T2=x(1+bezier_pts:end);
x = [-0.0238    0.5936    0.4211   -0.7933];
x= [-0.0394    0.6270    0.4255   -0.8444];
x=[-0.0445    0.6433    0.4351   -0.8695]; %Good initial guess for last system
x=[-0.0445    0.5    0.18   -0.9695]; % 
x=[0.0020    0.2088    0.4632   -0.5695];
% x=[0.0070    0.2050    0.4660   -0.5627]; %opt
x=[0.0070    0.2050    0.4660   -0.5627];
x=[ 0.1062    0.1918    0.2082   -0.4769]; %solution w new torque law
% x=[-0.0496    0.6421    0.4126   -0.8877];
x = [ 0.0070    0.2050    0.4660   -0.5627]; % energy = 0.0228, converged to infeasible point

ctrl.T1 = [.38 .38 -.5];
% x = [-0.05    0.5   -0.2];
x = [-0.049 0.61 -0.5];
% x = [-0.0393    0.4708   -0.1909]; % no objective function
% x = [-0.0705    0.3681   -0.0237]; % energy obj function for th1_0 = pi/4, dt = 0.001

% ctrl.T1=-3*[.07 .45 -.25];
% x=-[-.3 .55 0];

x = [0.0401    0.4217   -0.4086]; % minimize energy = 0.0153, T1 fixed
ctrl.T2 = x;
extra=[tf ctrl.tf ctrl.T1];

% optimizating both torque values
% th1_0 = pi/4, erergy = 0.0130
ctrl.T1 = [0.3440    0.3737   -0.4953]; 
ctrl.T2 = [0.0555    0.3786   -0.4141];

%% Run and animate simulation
[arm, pk, contact_pts, tspan, uout, energy]=simulate_system(z0, p, ctrl, tf, dt);
animate_system(arm, pk, contact_pts, p, tspan);