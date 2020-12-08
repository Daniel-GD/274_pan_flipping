setpath2
clear all; close all; clc;
% This is the main MATLAB script for Lab 4.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
const_point = [0.1;-.1]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
const_point(1)=-const_point(1);
pts_foot = repmat(const_point,1,8);

th1_0 = 0;

if th1_0 == 0   
%     T1=-[.07 .45 -.05];
%     T2=-[-.3 .55 0]*.6;
    T1=-[-0.0197    0.1121   -0.1579];
    T2=-[0.2444   -0.0741   -0.3151]*1.25; % from optimization
elseif th1_0 == pi/8
    T1 = -[-0.0241    0.2376   -0.5760];  
    T2 = -[0.3430   -0.1265   -0.5559]*1.25; % from optimization
elseif th1_0 == pi/6
%     T1=-[-.05 .20 -.05];
%     T2=-[-.15 .5 0];
    T1 = -[0.2424    0.1955   -0.2965]; %energy = 0.0065
    T2 = -[0.0897    0.2178   -0.1377]*1.25; % from optimization (works)
elseif th1_0 == pi/4
%     T1=-[.15 .25 -.05];
%     T2=-[-.2 .5 0];
    T1=-[0.3440    0.3737   -0.4953]; 
    T2=-[0.0555    0.3786   -0.4141]*1.25; % from optimization (works)
elseif th1_0 == 3*pi/8
    T1 = -[0.1930    0.3028   -0.5057];
    T2 = -[0.0915    0.4028   -0.9181]; % from optimization
elseif th1_0 == pi/2
%     T1=-[-.05 .35 0];
%     T2=-[-.15 .5 .0];
    T1=-[0.1259    0.2560   -0.5128];
    T2=-[0.1028    0.3722   -0.8829]*1.25;
end

ctrl.T1=T1; ctrl.T2=T2; ctrl.tf=.25;
torque_pts= [ctrl.T1; ctrl.T2];

% pts =[    0.1016    0.1016    0.1016    0.0678   -0.0561   -0.0619   -0.0619   -0.0619
%    -0.1102   -0.1102   -0.1102   -0.2048   -0.1511   -0.0436   -0.0436   -0.0436];
       
% pts_foot = [pts]; % YOUR BEZIER PTS HERE

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = 0;
if th1_0 == 0
    shoulder = 0;
    elbow = .5*pi;
elseif th1_0 == pi/8
    shoulder = -.1*pi;
    elbow = .43*pi;
elseif th1_0 == pi/6
    shoulder = -.15*pi;
    elbow = .42*pi;
elseif th1_0 == pi/4
    shoulder = -.32*pi;
    elbow = .32*pi;
elseif th1_0 == 3*pi/8
    shoulder = -.5*pi;
    elbow = .23*pi;
elseif th1_0 == pi/2
    shoulder = -.7*pi;
    elbow = 0.05*pi;
end
% Total experiment time is buffer,trajectory,buffer
traj_time         = ctrl.tf;
pre_buffer_time   = 3; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 1;
gains.K_yy = 1;
gains.K_xy = 0;

gains.D_xx = 0.05;
gains.D_yy = 0.05;
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 1;

%% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init, shoulder, elbow, torque_pts,...
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
tau1(1:10:end);
tau2(1:10:end);
final_pos=[x(end) y(end)];

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
% xline(pre_buffer_time);
% xline(pre_buffer_time+ctrl.tf);
xlim([(pre_buffer_time-.1) (pre_buffer_time+ctrl.tf+.1)]);
xlabel('Time (s)'); ylabel('T1'); legend({'Desired','Actual'});
title(['Optimized T1 Trajectory for ',num2str(th1_0),' Degrees'])

subplot(212); hold on
plot(t,tau2_des,'r-'); plot(t,tau2);
% xline(pre_buffer_time);
% xline(pre_buffer_time+ctrl.tf);
xlim([(pre_buffer_time-.1) (pre_buffer_time+ctrl.tf+.1)]);
xlabel('Time (s)'); ylabel('T2'); legend({'Desired','Actual'});
title(['Optimized T2 Trajectory for ',num2str(th1_0),' Degrees'])

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

