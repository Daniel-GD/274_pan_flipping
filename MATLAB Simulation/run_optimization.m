clear all; close all; clc;

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/4; %+.1;
% th2_0 = pi/4;%-pi/6;
th2_0 = -th1_0+pi/2;
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

%% Optimization
% Note: 5th state is the integral of torque squared over time
% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

% set guess
% tf = .5;                                        % simulation final time
tf = .5;                                        % guess for time minimization
dt = 0.00001;                                    % time step
ctrl.tf = 0.25;                                  % control time points
% ctrl.T1 = [.5 .5 -1];                             % control values
% ctrl.T2 = [.5 .5 -1];
% ctrl.T = [0 0 0];                               % guess for energy minimization
bezier_pts=3;
T1=[.5 .5 -.5];
% T2=[0.0020 0.2088 0.4632 -0.5695];
% T2=[0.0020    0.2088    0.4632   -0.5695]; %[-0.0445 0.5 0.18 -0.9695];
T2 = [0.1062 0.1918 -0.4769];
ctrl.T1=T1; ctrl.T2=T2; ctrl.tf=tf/2;

extra=[tf ctrl.tf ctrl.T1];
% x0 = [tf, ctrl.tf, ctrl.T1, ctrl.T2];
% x0 = x0(3:end);
x0 = ctrl.T2;
% % setup and solve nonlinear programming problem
% extra = [tf ctrl.tf];
% x0 = [ctrl.T1 ctrl.T2];

problem.objective = @(x) objective(x,z0,p,dt,extra);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints(x,z0,p,dt,extra);     % create anonymous function that returns nonlinear constraints
% problem.x0 = [tf ctrl.tf ctrl.T1 ctrl.T2];                   % initial guess for decision variables
problem.x0 = x0;
% problem.lb = [-2*ones(size(ctrl.T1)) -2*ones(size(ctrl.T2))];
% problem.ub = [2*ones(size(ctrl.T1)) 2*ones(size(ctrl.T2))]
problem.lb = [-2*ones(size(ctrl.T2))];
problem.ub = [2*ones(size(ctrl.T2))];
% problem.lb = [.1 .1 -bezier_pts*ones(size(ctrl.T1)) -bezier_pts*ones(size(ctrl.T2))];     % lower bound on decision variables
% problem.ub = [1  1   bezier_pts*ones(size(ctrl.T1))  bezier_pts*ones(size(ctrl.T2))];     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required

problem.options=optimoptions('fmincon','ConstraintTolerance', .05);
% options = optimoptions('fmincon','Display','iter');

x = fmincon(problem)                           % solve nonlinear programming problem

obj= objective(x,z0,p,dt,extra)

% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.
% tf=x(1);
% ctrl.tf=x(2);
% ctrl.T1=x(3:3+bezier_pts-1); %BUG HERE
% ctrl.T2=x(3+bezier_pts:end);

% ctrl.T1=x(1:1+bezier_pts-1); %BUG HERE
% ctrl.T2=x(1+bezier_pts:end);
ctrl.T1 = x(1:bezier_pts);
ctrl.T2 = x(1+bezier_pts:end);
ctrl;

% [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation
[arm, pk, contact_pts, tout, uout]=simulate_system(z0, p, ctrl, tf, dt);

% Torque_squared=sum(u.^2);
% COM_height=COM_jumping_leg(z(:,end),p);

%% Animate
animate_system(arm, pk, contact_pts, p, tout);