%% Runs entire simulation
setpath
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/4;%-pi/4; %+.1;
th2_0 = 0;%-th1_0+pi/2;%-pi/6;
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

M=A_arm(z0_arm,p.arm)
J=J_arm(z0_arm,p.arm)
% calculate the operational space mass matrix
LL = inv( (J/M)*J' )

% get eigenvectors and corresponding values
[V,D] = eig(LL); % columns of V are eigenvectors, diagonal elements of D are eigenvalues

% using parametric equations of ellipse, calculate ellipse points relative to foot position
th_ellipse = -atan2(V(1,1),V(2,1)); % angle between first eigenvector and positive x axis
l_x = 0.01*(1/D(1,1)); % TODO: better way to implement scaling of the ellipse?
l_y = 0.01*(1/D(2,2)); 
ii = linspace(0, 2*pi, 100);
xpts = (l_x*cos(ii))*cos(th_ellipse) - (l_y*sin(ii))*sin(th_ellipse);
ypts = (l_x*cos(ii))*sin(th_ellipse) + (l_y*sin(ii))*cos(th_ellipse);

inertia_ellipse = [xpts;ypts];
figure
plot(xpts,ypts)