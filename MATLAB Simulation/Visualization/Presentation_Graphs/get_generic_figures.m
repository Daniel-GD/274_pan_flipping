setpath
close all
%% Set parameters
p=parameters();

% Arm Initial Conditions
th1_0 = pi/2;%-pi/4; %+.1;
th2_0 = -th1_0+pi/2;%-pi/6;
% th2_0=pi/2;
dth1_0=0; %pi/2;
dth2_0=0 ;%2*pi;%2*pi;

z0_arm=[th1_0; th2_0; dth1_0; dth2_0];

% Pancake Initial Conditions
%Start the pancake at the end of the arm
pan_position= get_pan_position(z0_arm,p.arm); %Forward Kinematics of initial arm configuration
x0=pan_position(1,3)+.005; %place the pancake at the center of mass
y0=pan_position(2,3)+.025;
th0=pi/2+th1_0+th2_0 +pi-.3;
dx0=0; dy0=0; dth0=0;

% x0=pan_position(1,3); %place the pancake at the center of mass
% y0=pan_position(2,3)+.03;
% th0=pi/2+th1_0+th2_0 +.1;
% dx0=0; dy0=0; dth0=0;

contact_pts=0.*[1 1 1 1];

z0_pk=[x0; y0; th0; dx0; dy0; dth0];

p_arm=p.arm;
p_pk=p.pk;

z0= struct('arm',z0_arm,'pk',z0_pk);


%% Get Keypoints
    z_arm=z0_arm;
    z_pk=z0_pk;

    arm_keypoints = keypoints_arm(z_arm,p_arm);
    pk_keypoints = keypoints_pancake(z_pk,p_pk);
    pan_kpts=get_pan_position(z_arm,p_arm);

    %Get arm keypoints

    rc1 = arm_keypoints(:,1); % position vector to the CoM of link 1
    rB = arm_keypoints(:,2); %position vector to point B
    rc2 = arm_keypoints(:,3); % position vector to the CoM of link 2
    rpA = arm_keypoints(:,4); %position vector to point C
    rpan=pan_kpts(:,end);
   
    rC = arm_keypoints(:,5); %position vector to point C

    %Get pancake keypoints 
    rc_pk = pk_keypoints(:,1); % position vector to the CoM of pancake
    rA_pk = pk_keypoints(:,2); %position vector to point A in pancake
    rB_pk = pk_keypoints(:,3); % position vector to point B in pancake
    
 %% Plot
     figure; clf;
    % Prepare arm plot handles
    hold on
    plot(0,0,'.', 'MarkerSize' ,30,'MarkerEdgeColor','black');
    h_link1 = plot([0 rB(1)],[0 rB(2)],'b','LineWidth',10);
    h_link2 = plot([rB(1); rpA(1)],[rB(2); rpA(2)],'b','LineWidth',7);
    h_pan = plot([rpA(1); rC(1)],[rpA(2); rC(2)],'k','LineWidth',5);
    h_B_mark = plot(rB(1),rB(2),'.', 'MarkerSize' ,30,'MarkerEdgeColor','black');
    h_c1_mark = plot(rc1(1),rc1(2),'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    h_c2_mark = plot(rc2(1),rc2(2),'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    % Prepare pancake plot handle
    h_pk_line = plot([rA_pk(1); rB_pk(1)],[rA_pk(2); rB_pk(2)],'b','LineWidth',4);
    h_pk_line.Color=[181 101 30]./255;
    h_pk_com =plot([rc_pk(1)],[rc_pk(2)],'.', 'MarkerSize' ,20,'MarkerEdgeColor','black');
    
    % Prepare contact plot handles
%     h_contact1 =plot([rA_pk(1); rB_pk(1)],[rA_pk(2); rB_pk(2)],'.', 'MarkerSize' ,20,'MarkerEdgeColor','red');
%     h_contact2 =plot([rc_pk(1)],[rc_pk(2)],'.', 'MarkerSize' ,20,'MarkerEdgeColor','red');
    
    plot(0,0,'.', 'MarkerSize' ,35,'MarkerEdgeColor','black');
    xlabel('x [m]')
    ylabel('y [m]');
    h_title = title('Hybrid Simulation t=0.42s');
%     h_title = title('Hybrid Simulation t=0.42s');
    
    axis equal
    axis([-.1 .3 -.2 .2]);

J=J_pan_arm(z0_arm,p.arm)
J_scale=.6;
J=J_scale.*J;
hold on

J1=J(:,1);
J2=J(:,2);
J_sum=J1+J2
r_j=rpan;
quiver(r_j(1),r_j(2),J1(1),J1(2),'b')
quiver(r_j(1),r_j(2),J2(1),J2(2),'g')
quiver(r_j(1),r_j(2),J_sum(1),J_sum(2),'r')

% figure
thetas=[pi/2
3*pi/8
pi/4
pi/6
pi/8
pi*0];

% thetas=sin(thetas)
energies=[0.036
0.057
0.013
0.0065
0.0052
0.002];

% thetas=thetas(2:end);
% energies=energies(2:end);

[p,S]=polyfit(thetas,energies,1);
x1 = linspace(thetas(1),thetas(end));
y1 = polyval(p,x1);

figure

plot(thetas,energies,'.b','MarkerSize',20)
title("Energies vs Initial Angle")
xlabel('Initial Angle [rads]')
ylabel('Energy Approximation [(Nm)^2]')
hold on
plot(x1,y1,'r--','LineWidth',2)