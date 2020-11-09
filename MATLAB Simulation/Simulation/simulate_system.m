function [arm, pk, contact_pts, tspan]=simulate_system(z0, p, u, tf, dt)
% [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,tspan)
% Hybrid simulation of the entire system Arm + Pancake for a series of
% inputs
% z0 p are structs
% u is supposed to be an array of torques applied to the arm (rn it is not)
% Returns two structs with components z_out and u_out

setpath

%% Perform Dynamic simulation    
% dt = 0.00001;
% tf = 10;
num_steps = floor(tf/dt);
tspan = linspace(0, tf, num_steps); 

%Initialize variables
z_out_arm = zeros(numel(z0.arm),num_steps);
z_out_pk = zeros(numel(z0.pk),num_steps);
contact_pts=zeros(4,num_steps);

z_out_arm(:,1) = z0.arm;
z_out_pk(:,1) = z0.pk;

u_pk=[0;0;0]; %Initialize u 

% z_out = zeros(4,num_steps);
% z_out(:,1) = z0;
for i=1:num_steps-1
    %Move arm
    z_arm= step_arm(z_out_arm(:,i),p.arm, u(:,i), dt);
    %Move pancake Should we move pancake before applying force
    z_pk=step_pancake(z_out_pk(:,i),p.pk, u_pk, dt);
    
    %%%%% LAURA %%%%%
    %Check collision (CONTACT MODELING) Calculate Fx, Fy and Tau
%     u_pk=[0;0;0];
%     if sum(z_arm(1:2))<pi/2 +.01
    [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p);
%     end
    %%%%% LAURA %%%%%
    
    %Apply new torque to pancake
    z_pk=step_pancake(z_out_pk(:,i),p.pk, u_pk, dt);

    
    %Add final states to arrays
    z_out_arm(:,i+1) = z_arm;
    z_out_pk(:,i+1) = z_pk;
    contact_pts(:,i+1) = p_contact;
end
arm.z_out=z_out_arm;
pk.z_out=z_out_pk;
end