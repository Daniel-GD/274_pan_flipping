%% Runs entire simulation
setpath
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/4; %+.1;
th2_0 = pi/6;%-pi/6;
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
num_steps = floor(tf/dt);
timestop=round(1*num_steps/3);
u=zeros(2,num_steps);
u(1,1:timestop)=1;
u(2,1:timestop)=1;
u(1,timestop+1: end)=-5;
u(2,timestop+1: end)=-5;

%% Run and animate simulation
[arm, pk, contact_pts, tspan]=simulate_system(z0, p, u, tf, dt);
animate_system(arm, pk, contact_pts, p, tspan);