function [cineq, ceq] = constraints(x,z0,p,dt,extra)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

    % constraints
    
    tol1=.01;
    % x = [tf, ctrl.tf, ctrl.T]; i added a dt to the x struc
    %run simulation
    bezier_pts=4;
%     tf=x(1);
%     ctrl.tf=x(2);
%     ctrl.T1=x(3:3+bezier_pts-1); %BUG HERE
%     ctrl.T2=x(3+bezier_pts:end);
    tf=extra(1);
    ctrl.tf=extra(2);
%     ctrl.T1=x(1:1+bezier_pts-1); %BUG HERE
%     ctrl.T2=x(1+bezier_pts:end);    
    ctrl.T1 = extra(3:end);
    ctrl.T2 = x;
    
    
    % z0 is a struct with arm & pk
    [arm, pk, contact_pts, tout, uout, energy]=simulate_system(z0, p, ctrl, tf, dt);
%     animate_system(arm, pk, contact_pts, p, tspan);
    z_out_arm = arm.z_out;
    z_out_pk = pk.z_out;
    z_out_pk(:,end);
    %Pan must flip
    % final angle must be 180 deg -- we could also change this to any
    % multiple of 180?
    pk_th0=z0.pk(3);
    pk_thf=z_out_pk(3,end);
    delta_th=-(pk_thf-pk_th0);
    delta_th=abs(pk_thf-pk_th0);
    cineq_flip = pi-delta_th;
    
    cineq_flip_too_much = delta_th-2*pi;
    

    
    
    %Pan must not fall from the pan
    pk_final_com=z_out_pk(1:2,end); %2x1 vector
    %Get left of pan
    pan_final_positions=get_pan_position(z_out_arm(:,end),p.arm); % 2x3 vector
    cineq_fall_left=pan_final_positions(1,1)-pk_final_com(1); %edge of the pan
%     cineq_fall_left=pan_final_positions(1,3)-pk_final_com(1); %com of pan
    cineq_fall_right=pk_final_com(1)-pan_final_positions(1,2); %right edge of pan
    
    cineq =[cineq_flip cineq_flip_too_much cineq_fall_left cineq_fall_right];
    cineq= [cineq_fall_left cineq_fall_right];
%     cineq =[cineq_flip  cineq_fall_side];
    
    ceq=[delta_th-pi];
    %Rightmost point of pancake < rightmost point of pan
    % get pan rightmost position
%     arm_kpts = keypoints_arm(z0.arm,p.arm);
%     pk_kpts = keypoints_pancake(z0.pk,p.pk);
    
%     size(arm_kpts)
%     size(pk_kpts)
%     arm_kpts
%     pk_kpts
%     size(arm.z_out)
%     size(pk.z_out)
    
    % comparing x points
%     cineq1 = arm_kpts(1,end)-pk_kpts(1,end);
%     
%     % we dont want the arm to overextend
%     cineq2 = max(arm.z_out(1,:))-pi/2; % don't overextend first arm, th1
%     cineq3 = max(arm.z_out(2,:))-pi/2; % don't overextend second arm, th2
%     
%     cineq = [cineq1, cineq2, cineq3];
    %Torque limits
    
    %Joint limits
    
%     tf=x(1);
%     ctrl.tf=x(2);
%     ctrl.T=x(3:end);
%     tspan= [0 tf];
%     [tout, zout,~, indices]=hybrid_simulation(z0,ctrl,p,tspan);
    
%     COMs=COM_jumping_leg(zout,p);
%     f= max(COMs(2,:));
%     apex_c= .4-f;
%     
%     cineq1=-min(zout(2,:));
%     cineq2 =max(zout(2,:))-pi/2;
    
%     cineq = [cineq1 , cineq2]; 
%     cineq = [cineq1 , cineq2, apex_c];  

%     if indices(1)==0 %Check if you actually took off
%         indices(1)=1;
%     end
%     takeoff_ceq=ctrl.tf-tout(indices(1));
%          ceq = [ takeoff_ceq];  
%     
%     ceq = [ takeoff_ceq apex_c]; 
%     
                                                            
% simply comment out any alternate constraints when not in use
    
end