% This is the main MATLAB script for Lab 4.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
const_point = [0.1;-.1]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
const_point(1)=-const_point(1);
pts_foot = repmat(const_point,1,8);


T1=-[.5 0 -.5 -.5];
T2=-[-.1 .35 .5 -1];
ctrl.T1=T1; ctrl.T2=T2; ctrl.tf=.25;

torque_pts= [ctrl.T1;ctrl.T2];

% pts =[    0.1016    0.1016    0.1016    0.0678   -0.0561   -0.0619   -0.0619   -0.0619
%    -0.1102   -0.1102   -0.1102   -0.2048   -0.1511   -0.0436   -0.0436   -0.0436];
       
% pts_foot = [pts]; % YOUR BEZIER PTS HERE

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = 0;

% Total experiment time is buffer,trajectory,buffer
traj_time         = ctrl.tf;
pre_buffer_time   = 3; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 5;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 2;
gains.K_yy = 2;
gains.K_xy = 0;

gains.D_xx = 0.01;
gains.D_yy = 0.01;
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = .3;

%% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init, torque_pts,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       gains, duty_max);

%% Extract data
t = output_data(:,1);
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
   
% xdes = output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
% ydes = output_data(:,17); % desired foot position in Y
tau1 = output_data(:,16);      % desired foot position (negative due to direction motors are mounted)
tau2 = output_data(:,17);      % desired foot position        
tau1_des = output_data(:,18);      % desired foot position (negative due to direction motors are mounted)
tau2_des = output_data(:,19);      % desired foot position   

final_pos=[x(end) y(end)]

%% Plot foot vs desired
figure(3); clf;
% subplot(211); hold on
% plot(t,xdes,'r-'); plot(t,x);
% xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});
% 
% subplot(212); hold on
% plot(t,ydes,'r-'); plot(t,y);
% xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});
subplot(211); hold on
plot(t,tau1_des,'r-'); plot(t,tau1);
xlabel('Time (s)'); ylabel('TAU1'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,tau2_des,'r-'); plot(t,tau2);
xlabel('Time (s)'); ylabel('TAU2'); legend({'Desired','Actual'});

% figure(4); clf; hold on
% plot(xdes,ydes,'r-'); plot(x,y,'k');
% xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});

% for Cartesian constant points, un-comment this to see the virtual potential well on figure 4
% [X, Y] = meshgrid(linspace(-0.25,0.25,50),linspace(-0.25,0.1,50));
% eX = X - const_point(1); 
% eY = Y - const_point(2); 
% V = 0.5*gains.K_xx*eX.*eX + 0.5*gains.K_yy*eY.*eY + gains.K_xy*eX.*eY;
% axis([-0.25, 0.25, -0.25, 0.1]);
% contour(X,Y,V,15,'LineWidth',1.5);

