function [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p)
% Takes in current state of arm-pancake system
% Returns Generalized Forces u_pk=[Fx Fy Tau] to apply to the pancake
% It might need to change velocity? Don't remember

%Normal direction of the contact surface
%Separate cases
% Collinear
% Edge of the pan
% Prohibi t normal direction penetration
% Find normal direction, check for two points
Fx=0;
Fy=0;
Tau=0;
% [u_pk, p_contact] = discrete_impact_contact_edges(z_arm,z_pk,p, 10000, 20);
[u_pk, p_contact] = discrete_impact_contact(z_arm,z_pk,p, 1000, 20);
end

function t=get_t(x,x0,xf)  %Inverse linear interpolation
    t=(x(1)-x0(1))/(xf(1)-x0(1));
    if t==inf 
        t=(x(2)-x0(2))/(xf(2)-x0(2));
    end 
end

function [u_pk, p_contact] = discrete_impact_contact(z_arm,z_pk,p, K_c, fric_coeff)
    p_contact=NaN(1,4);
%     K_c=1000; %10000;
    D_c=20; %.8;
    tau_k=1;
    
    %Get keypoints
    pan_position= get_pan_position(z_arm,p.arm);
    pk_position= get_pancake_position(z_pk,p.pk);
    pk_keypoints= keypoints_pancake(z_pk,p.pk);
    
    rA=pk_position(:,1);
    rB=pk_position(:,2);
    rC=pk_keypoints(:,1);
    
    %Get keypoint velocities
    pk_velocity= get_pancake_velocity(z_pk,p.pk);
    drA=pk_velocity(:,1);
    drB=pk_velocity(:,2);
    
    %Get Vectors
    e2hat=pan_normal(z_arm,p.arm);
    e1hat=pan_parallel(z_arm,p.arm); %parallel to pan
    e_pk=pk_parallel(z_pk,p.pk);
    
    %Get Projected Points
    pA=rA-dot(rA-pan_position(:,1),e2hat)*e2hat;
    pB=rB-dot(rB-pan_position(:,1),e2hat)*e2hat;
    
    %Get intersection velocities
    tA=get_t(pA,pan_position(:,1),pan_position(:,2));
    dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    tB=get_t(pB,pan_position(:,1),pan_position(:,2));
    dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    
    %1st constraint
    FcA=0;
    if tA>=0 && tA<=1 %Check that pancake is within pan
        CA=dot(rA-pA,e2hat);
        CA_dot=dot(drA-dpA,e2hat);
        if CA<0 %penetrating
            if CA>-.08 %Pancake already left the pan and is just
                p_contact(1:2)=pA;
                FcA=-K_c*CA-D_c*CA_dot; %Apply a force at that point
            end
        end
    end
    FA=FcA*e2hat;
    tauA=cross([(rA-rC); 0],[FA; 0]);
    
    %2nd constraint
    FcB=0;
    if tB>=0 && tB<=1         
        CB=dot(rB-pB,e2hat);
        CB_dot=dot(drB-dpB,e2hat);
        if CB<0
            if CB>-.08
                p_contact(3:4)=pB;
                FcB=-K_c*CB-D_c*CB_dot; %Apply a force at that point
            end
        end
    end
    FB=FcB*e2hat;   
    tauB=cross([(rB-rC); 0],[FB; 0]);
   
    %Add Forces
    F=FA+FB;
    tau=tau_k*(tauA(3)+tauB(3));

    u_pk=[F(1); F(2); tau];
end

function [u_pk, p_contact] = discrete_impact_contact_edges(z_arm,z_pk,p, K_c, fric_coeff)
    p_contact=NaN(1,4);
%     K_c=100; %10000;
    D_c=20; %.8;
    tau_k=1;
    
    %Get keypoints
    pan_position= get_pan_position(z_arm,p.arm);
    pk_position= get_pancake_position(z_pk,p.pk);
    pk_keypoints= keypoints_pancake(z_pk,p.pk);
    
    rA=pk_position(:,1);
    rB=pk_position(:,2);
    rC=pk_keypoints(:,1);
    
    %Get keypoint velocities
    pk_velocity= get_pancake_velocity(z_pk,p.pk);
    drA=pk_velocity(:,1);
    drB=pk_velocity(:,2);
    
    %Get Vectors
    e2hat=pan_normal(z_arm,p.arm);
    e1hat=pan_parallel(z_arm,p.arm); %parallel to pan
    e_pk=pk_parallel(z_pk,p.pk);
    
    %Get Projected Points
    pA=rA-dot(rA-pan_position(:,1),e2hat)*e2hat;
    pB=rB-dot(rB-pan_position(:,1),e2hat)*e2hat;
    
    %Get intersection velocities
    tA=get_t(pA,pan_position(:,1),pan_position(:,2));
    dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    tB=get_t(pB,pan_position(:,1),pan_position(:,2));
    dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    
    %First Constraint
    if tA>=0 && tA<=1
        %You know what to do
    elseif tA<0 && tB>=0
        %rA is over left edge of pan
        tA=0;
        pA=pan_position(:,1);
        dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    elseif tA>1 && tB<=1
        %rA is over right edge of pan
        tA=1;
        pA=pan_position(:,2);
        dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    else
        rA(:)=0;
        pA(:)=0;
    end

    FcA=0;
    CA=dot(rA-pA,e2hat);
    CA_dot=dot(drA-dpA,e2hat);
    if CA<0 %penetrating
        if CA>-.08 %Pancake already left the pan and is just align w pan
            p_contact(1:2)=pA;
            FcA=-K_c*CA-D_c*CA_dot; %Apply a force at that point
        end
    end
    FA=FcA*e2hat;
    tauA=cross([(rA-rC); 0],[FA; 0]);

    %Second Constraint
    if tB>=0 && tB<=1
        %You know what to do
    elseif tB<0 && tA>=0
        %rB is over left edge of pan
        tB=0;
        pB=pan_position(:,1);
        dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    elseif tB>1 && tA<=1
        %rB is over right edge of pan
        tB=1;
        pB=pan_position(:,2);
        dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    else
        rB(:)=0;
        pB(:)=0;
    end

    FcB=0;
    CB=dot(rB-pB,e2hat);
    CB_dot=dot(drB-dpB,e2hat);
    if CB<0 %penetrating
        if CB>-.08 %Pancake already left the pan and is just align w pan
            p_contact(3:4)=pB;
            FcB=-K_c*CB-D_c*CB_dot; %Apply a force at that point
        end
    end
    FB=FcB*e2hat;   
    tauB=cross([(rB-rC); 0],[FB; 0]);
   
    %Add Forces
    F=FA+FB;
    tau=tau_k*(tauA(3)+tauB(3));

    u_pk=[F(1); F(2); tau];
end

function [u_pk, p_contact] = discrete_impact_contact_dynamics(z_arm,z_pk,p, rest_coeff, fric_coeff)
    p_contact=NaN(1,4);
    K_c=10000;
    D_c=.8;
    %Get keypoints
    pan_position= get_pan_position(z_arm,p.arm);
    
    pk_position= get_pancake_position(z_pk,p.pk);
    pk_keypoints= keypoints_pancake(z_pk,p.pk);
    rA=pk_position(:,1);
    
    rB=pk_position(:,2);
    rC=pk_keypoints(:,1);
    
    %Get keypoint velocities
    pk_velocity= get_pancake_velocity(z_pk,p.pk);
    drA=pk_velocity(:,1);
    drB=pk_velocity(:,2);
    
    %Get Vectors
    e2hat=pan_normal(z_arm,p.arm);
    e1hat=pan_parallel(z_arm,p.arm); %parallel to pan
    e_pk=pk_parallel(z_pk,p.pk);
    
    %Get Projected Points
    pA=rA-dot(rA-pan_position(:,1),e2hat)*e2hat;
    pB=rB-dot(rB-pan_position(:,1),e2hat)*e2hat;
    
    %Get intersection velocities
    tA=get_t(pA,pan_position(:,1),pan_position(:,2));
    dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    tB=get_t(pB,pan_position(:,1),pan_position(:,2));
    dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    
    %1st constraint
    CA=dot(rA-pA,e2hat);
    CA_dot=dot(drA-dpA,e2hat);
    FcA=0;
    if CA<0
            p_contact(1:2)=pA;
            FcA=-K_c*CA-D_c*CA_dot; %Apply a force at that point
    end
    FA=FcA*e2hat;
    tauA=cross([(rA-rC); 0],[FA; 0]);
    
    %2nd constraint
    CB=dot(rB-pB,e2hat);
    CB_dot=dot(drB-dpB,e2hat);
    FcB=0;
    if CB<0
        p_contact(3:4)=pB;
        FcB=-K_c*CB-D_c*CB_dot; %Apply a force at that point
    end
    FB=FcB*e2hat;
    tauB=cross([(rB-rC); 0],[FB; 0]);
    
    
    if CA<0 && CA_dot<0 %if pancake point is below the pan and velocity is pointing below the pan   
        %Get Jacobians
        JA=jacobian_pk_A(z,p);
        JA_cx=JA(1,:);
        JA_cz=JA(2,:);
        
        %Get Mass Matrix
        M=A_pancake(z,p); %mass matrix
        M_inv=inv(M); %store mass matrix inverse
        
                % Compute Operational Space Mass Matrices
        lambdaA= inv(JA*M_inv*JA');    % Operational Space Mass Matrix
        lambdaA_cx=inv(JA_cx*M_inv*JA_cx');
        lambda_cz=inv(JA_cz*M_inv*JA_cz');
        FA_cz=lambda_cz*(-rest_coeff*C_dot-J_cz*qdot); 
        
        %Update joint velocities based on vertical force (normal force)
        qdot=qdot+M_inv*J_cz'*F_cz; 
        
        F_cx=lambda_cx*(0-J_cx*qdot);
        if F_cx>fric_coeff*F_cz
            F_cx=fric_coeff*F_cz;
        elseif F_cx<-fric_coeff*F_cz
            F_cx=-fric_coeff*F_cz;
        end
        
        %Update joint velocities based on horizontal force (friction)
        qdot=qdot+M_inv*J_cx'*F_cx; 
    end
    %     if CA<0 && CA_dot<0 %if pancake point is below the pan and velocity is pointing below the pan        
%         % Get Jacobians
%         J  = jacobian_pk(z,p);
%         J_cx=J(1,:);
%         J_cz=J(2,:);
% 
%         %Get Mass Matrix
%         M=A_pancake(z,p); %mass matrix
%         M_inv=inv(M); %store mass matrix inverse
% 
%         
%         % Compute Operational Space Mass Matrices
%         lambda= inv(J*M_inv*J');    % Operational Space Mass Matrix
%         lambda_cx=inv(J_cx*M_inv*J_cx');
%         lambda_cz=inv(J_cz*M_inv*J_cz');
%         F_cz=lambda_cz*(-rest_coeff*C_dot-J_cz*qdot); 
%         
%         %Update joint velocities based on vertical force (normal force)
%         qdot=qdot+M_inv*J_cz'*F_cz; 
%         
%         F_cx=lambda_cx*(0-J_cx*qdot);
%         if F_cx>fric_coeff*F_cz
%             F_cx=fric_coeff*F_cz;
%         elseif F_cx<-fric_coeff*F_cz
%             F_cx=-fric_coeff*F_cz;
%         end
%         
%         %Update joint velocities based on horizontal force (friction)
%         qdot=qdot+M_inv*J_cx'*F_cx; 
%     end
%     
%     
%     %Calculate end effector position and velocity
%     rE=position_foot(z,p); %get forward kinematics of end point
%     vE=velocity_foot(z,p); %get velocity of end point
%     
%     C=rE(2)-yC; %get relative height
%     C_dot=vE(2); %C_dot is just y_dot
%     
%     qdot=z(3:4); %Initialize joint velocities
%        
%     if C<0 && C_dot<0 %if foot is below the ground and velocity is pointing down        
%         % Get Jacobians
%         J  = jacobian_foot(z,p);
%         J_cx=J(1,:);
%         J_cz=J(2,:);
% 
%         %Get Mass Matrix
%         M=A_leg(z,p); %mass matrix
%         M_inv=inv(M); %store mass matrix inverse
% 
%         % Compute Operational Space Mass Matrices
%         lambda= inv(J*M_inv*J');    % Operational Space Mass Matrix
%         lambda_cx=inv(J_cx*M_inv*J_cx');
%         lambda_cz=inv(J_cz*M_inv*J_cz');
%         F_cz=lambda_cz*(-rest_coeff*C_dot-J_cz*qdot); 
%         
%         %Update joint velocities based on vertical force (normal force)
%         qdot=qdot+M_inv*J_cz'*F_cz; 
%         
%         F_cx=lambda_cx*(0-J_cx*qdot);
%         if F_cx>fric_coeff*F_cz
%             F_cx=fric_coeff*F_cz;
%         elseif F_cx<-fric_coeff*F_cz
%             F_cx=-fric_coeff*F_cz;
%         end
%         
%         %Update joint velocities based on horizontal force (friction)
%         qdot=qdot+M_inv*J_cx'*F_cx; 
%     end
    
    %Add Forces
    F=FA+FB;
    tau=tauA(3)+tauB(3);

    u_pk=[F(1); F(2); tau];
    
    
end