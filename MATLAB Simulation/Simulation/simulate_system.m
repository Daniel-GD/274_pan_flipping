function [arm, pk, contact_pts, tout, uout, energy]=simulate_system(z0, p, ctrl, tf, dt)
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
energy = 0;

z_out_arm(:,1) = z0.arm;
z_out_pk(:,1) = z0.pk;

u_pk=[0;0;0]; %Initialize u 

state=0; 
left_pan=false;
contact=false;
last_contact=false;
iphase_list = 1;
complex_contact=false;
% z_out = zeros(4,num_steps);
% z_out(:,1) = z0;
for i=1:num_steps-1
    %Get input torque for the arm
    
    u = control_laws(tout(i),z_out_arm(:,i),p,ctrl,state,z_out_pk(:,i));
    
    %Move arm
    z_arm= step_arm(z_out_arm(:,i),p.arm, u, dt);
    %Move pancake Should we move pancake before applying force
    z_pk=step_pancake(z_out_pk(:,i),p.pk, u_pk, dt);
    
    %%%%% LAURA %%%%%
    %Check collision (CONTACT MODELING) Calculate Fx, Fy and Tau
%     u_pk=[0;0;0];
%     if sum(z_arm(1:2))<pi/2 +.01
    [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p,complex_contact);
    
    if sum(isnan(p_contact))==4
        contact=false;
    else
        contact=true;
    end
    if last_contact ~=contact
        state=state+1;
    end
    
    if state<2
        energy = energy + (u(1)^2 + u(2)^2)*dt;
%     else
%         if ~left_pan
%             left_pan=true;
%             tout(i)
%         end
%     else
%         complex_contact=true;
    end
    if tout(i)>ctrl.tf
        complex_contact=true;
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
function u = control_laws(t,z,p,ctrl,state,z_pk)
    if state <2 && t<ctrl.tf %pancake is in contact w the pan
        u1 = BezierCurve(ctrl.T1, t/ctrl.tf);
        u2 = BezierCurve(ctrl.T2, t/ctrl.tf);
    else %pancake is in flight -> control law
        %Go back to some desired position
        th1 = z(1,:);
        th2 = z(2,:);  
        dth1 = z(3,:);     
        dth2 = z(4,:);           % leg angular velocity

        th1_d = -pi/6;             % desired leg angle
        th2_d = pi/6+pi/2;
        k1 = 5;  k2 = 5;                % stiffness (N/rad)
        b1 = .5; b2 = .5;                % damping (N/(rad/s))
        
        %Apply joint pd control
        u1 = -k1*(th1-th1_d) - b1*dth1;% apply PD control
        u2 = -k2*(th2-th2_d) - b2*dth2;% apply PD control
        
        %should take in pancake positon in x
        %should convert that to desired th1 and th2 should be calculated
        %based on that
        k1 = 20;  k2 = 20;                % stiffness (N/rad)
%         k1 = 15;  k2 = 15;                % stiffness (N/rad)
        p_arm=p.arm;
        l1=p_arm(3);
        c=p_arm(end)+(p_arm(4)-p_arm(end))/2+.03; %I have no idea why .03 works
        x=z_pk(1)-c;
        e_x=x-l1*sin(th1);
        if e_x>l1
            e_x=l1;
        elseif e_x<-l1
            e_x=-l1;
        end
        th1_d=asin(e_x/l1);
        
        th2_d=-th1_d+pi/2;
        u1 = -k1*(th1-th1_d) - b1*dth1;% apply PD control
        u2 = -k2*(th2-th2_d) - b2*dth2;% apply PD control
%         return
        
    end
    u=[u1;u2];

end