function [arm, pk, contact_pts, tout, uout]=simulate_system(z0, p, ctrl, tf, dt)
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
tout = linspace(0, tf, num_steps); 

%Initialize variables
z_out_arm = zeros(numel(z0.arm),num_steps);
z_out_pk = zeros(numel(z0.pk),num_steps);
contact_pts=zeros(4,num_steps);
uout = zeros(5, num_steps);

z_out_arm(:,1) = z0.arm;
z_out_pk(:,1) = z0.pk;

u_pk=[0;0;0]; %Initialize u 

state=0; 
contact=false;
last_contact=false;
iphase_list = 1;
% z_out = zeros(4,num_steps);
% z_out(:,1) = z0;
for i=1:num_steps-1
    %Get input torque for the arm
    
    u = control_laws(tout(i),z_out_arm(:,i),ctrl,state);
    
    %Move arm
    z_arm= step_arm(z_out_arm(:,i),p.arm, u, dt);
    %Move pancake Should we move pancake before applying force
    z_pk=step_pancake(z_out_pk(:,i),p.pk, u_pk, dt);
    
    %%%%% LAURA %%%%%
    %Check collision (CONTACT MODELING) Calculate Fx, Fy and Tau
%     u_pk=[0;0;0];
%     if sum(z_arm(1:2))<pi/2 +.01
    [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p);
    
    if sum(isnan(p_contact))==4
        contact=false;
    else
        contact=true;
    end
    if last_contact ~=contact
        state=state+1;
    end
    
%     end
    %%%%% LAURA %%%%%
    
    %Apply new torque to pancake
    z_pk=step_pancake(z_out_pk(:,i),p.pk, u_pk, dt);

    
    %Add final states to arrays
    z_out_arm(:,i+1) = z_arm;
    z_out_pk(:,i+1) = z_pk;
    contact_pts(:,i+1) = p_contact;
    uout(:,i+1) = [u; u_pk];
    last_contact=contact;
end
state;
arm.z_out=z_out_arm;
pk.z_out=z_out_pk;
end

%% Control
function u = control_laws(t,z,ctrl,state)
    if state <2 && t<ctrl.tf %pancake is in contact w the pan
        u1 = BezierCurve(ctrl.T1, t/ctrl.tf);
        u2 = BezierCurve(ctrl.T2, t/ctrl.tf);
    else %pancake is in flight
        %Go back to some desired position
        th1 = z(1,:);
        th2 = z(2,:);  
        dth1 = z(3,:);     
        dth2 = z(4,:);           % leg angular velocity

        th1_d = -pi/6;             % desired leg angle
        th2_d = pi/6+pi/2;
        k1 = 40;  k2 = 40;                % stiffness (N/rad)
        k1 = 5;  k2 = 5;                % stiffness (N/rad)
        b1 = .5; b2 = .5;                % damping (N/(rad/s))
        
        %Apply joint pd control
        u1 = -k1*(th1-th1_d) - b1*dth1;% apply PD control
        u2 = -k2*(th2-th2_d) - b2*dth2;% apply PD control
    end
    u=[u1;u2];

end