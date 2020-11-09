function [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p)
% Takes in current state of arm-pancake system
% Returns Generalized Forces [Fx Fy Tau] to apply to the pancake
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
% p_intersect= find_intersection(z_arm,z_pk, p);
% if numel(p_intersect) ==0
%     %no contact
%     Fx=0;
%     Fy=0;
%     Tau=0;
%     p_contact=[NaN NaN];
%     [Fx, Fy, Tau] = discrete_impact_contact_center(z_arm,z_pk,p,p_contact, 1000, 20);
% elseif numel(p_intersect(1,:)) ==1
%    % Intersection
%    p_contact=p_intersect;
%    [Fx, Fy, Tau] = discrete_impact_contact_int(z_arm,z_pk,p,p_contact, 1000, 20);
%    
%    
% elseif numel(p_intersect(1,:))==2
%     % Colinear
%     p_contact=mean(p_intersect);
% %     [Fx, Fy, Tau] = discrete_impact_contact(z_arm,z_pk,p,p_contact, 1000, 20);
%     %It shouldnt apply a torque right?
% end
[u_pk, p_contact] = discrete_impact_contact(z_arm,z_pk,p, 1000, 20);
% p_contact=[0;0];
% u_pk=[Fx; Fy; Tau];
end

function [p_intersection]= find_intersection(z_arm,z_pk, p)
%FK
arm_keypoints= keypoints_arm(z_arm,p.arm);
pk_keypoints= keypoints_pancake(z_pk,p.pk);
%Get arm keypoints
rc1 = arm_keypoints(:,1); % position vector to the CoM of link 1
rB = arm_keypoints(:,2); %position vector to point B
rc2 = arm_keypoints(:,3); % position vector to the CoM of link 2
rC = arm_keypoints(:,4); %position vector to point C

%Get pancake keypoints 
rc_pk = pk_keypoints(:,1); % position vector to the CoM of pancake
rA_pk = pk_keypoints(:,2); %position vector to point A in pancake
rB_pk = pk_keypoints(:,3); % position vector to point B in pancake

x_pan=[rB(1) rC(1)];
y_pan=[rB(2) rC(2)];
x_pk=[rA_pk(1) rB_pk(1)];
y_pk=[rA_pk(2) rB_pk(2)];

[xi,yi] = polyxpoly(x_pan,y_pan,x_pk,y_pk);

p_intersection=[xi; yi];
end

% function [p_intersection, t_pan t_pancake]= find_intersection(z_arm,z_pk, p)
% p_intersection=[NaN; NaN];
% %Forward Kinematics
% arm_keypoints= keypoints_arm(z_arm,p.arm);
% pk_keypoints= keypoints_pancake(z_pk,p.pk);
% %Get arm keypoints
% rc1 = arm_keypoints(:,1); % position vector to the CoM of link 1
% rB = arm_keypoints(:,2); %position vector to point B
% rc2 = arm_keypoints(:,3); % position vector to the CoM of link 2
% rC = arm_keypoints(:,4); %position vector to point C
% 
% %Get pancake keypoints 
% rc_pk = pk_keypoints(:,1); % position vector to the CoM of pancake
% rA_pk = pk_keypoints(:,2); %position vector to point A in pancake
% rB_pk = pk_keypoints(:,3); % position vector to point B in pancake
% 
% % From https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
% p=rA_pk;
% r=rB_pk-rA_pk;
% 
% q=rB;
% s=rC-rB;
% qp=q-p;
% % c_qps=cross((q-p),s);
% c_qps=qp(1)*s(2)-qp(2)*s(1);
% % c_qpr=cross((q-p),r);
% c_qpr=qp(1)*r(2)-qp(2)*r(1);
% % c_rs=cross(r,s);
% c_rs= r(1)*s(2)-r(2)*s(1);
% t=c_qps/c_rs;
% u=c_qpr/c_rs;
% 
% tol=1e-4;
% if abs(c_rs)<tol 
%     if abs(c_qpr)<tol
%         %collinear (no moment)
%         %Check end points
%         
%         rr=dot(r,r);
%         t0= dot(qp,r)/rr;
%         t1=dot((q+s-p),r)/rr;
%         t0t1=[t0 t1]
%         if t>=0 && t<=1 && u>=0 && u<=1
%         p_intersection=[0,.1];
%         
%         end
%         %"colinear"
%     else
%         %Two lines are parallel and not intersecting
%     end
% else
%     if t>=0 && t<=1 && u>=0 && u<=1
%         %Line is intersecting and notn parallel
%         %Line meets a th the point p+tr = q +us
%         p_intersection=p+t.*r;
% %         [t u]
%         
%     else
%         %Not parallel and not intersecting
%     end
%     
% end
% 
% t_pancake=u;
% t_pan=t;
% end
function t=get_t(x,x0,xf)
    
%     T=(x-x0)./(xf-x0);
%     t=min(T);
%     
%     if T(1)==Inf || T(2) ==Inf
%         "HELLOOOO"
%     end
%     T=(find(T~=Inf));
%     t=min(find(T~=Inf));

    
    t=(x(1)-x0(1))/(xf(1)-x0(1));
    if t==inf 
        t=(x(2)-x0(2))/(xf(2)-x0(2));
    end
        
end

function [Fx, Fy, Tau] = discrete_impact_contact_center(z_arm,z_pk,p,p_int, rest_coeff, fric_coeff)
    K_c=10000;
    D_c=20;
    %Get keypoints
    pan_position= get_pan_position(z_arm,p.arm);
    
    pk_position= get_pancake_position(z_pk,p.pk);
    pk_keypoints= keypoints_pancake(z_pk,p.pk);
    rC=pk_keypoints(:,1);
    

    
    %Get keypoint velocities
    drC=get_pancake_com_velocity(z_pk,p.pk);
    
    %Get Vectors
    e2hat=pan_normal(z_arm,p.arm);
    
    %Get "intersection" point
    p_int=rC+dot(rC-pan_position(:,1),e2hat)*e2hat;
    
    %Get intersection velocities
    t_pan= get_t(p_int,pan_position(:,1),pan_position(:,2));
    dp_int=pan_velocity_interpolation(z_arm,p.arm,t_pan);

    %1st constraint
    C=-dot(rC-p_int,e2hat);
    C_dot=dot(drC-dp_int,e2hat);
    Fc=0;
    if C<0
        Fc=-K_c*C-D_c*C_dot; %Apply a force at that point
    end
    F=Fc*e2hat;
    Fx=F(1);
    Fy=F(2);

    Tau=0;
end

% function qdot = discrete_impact_contact(z,p, rest_coeff, fric_coeff, yC)
function [Fx, Fy, Tau] = discrete_impact_contact_int(z_arm,z_pk,p,p_int, rest_coeff, fric_coeff)
    K_c=10000;
    D_c=20;
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
 
    
    
    %Calculate intersection
%     p_int=find_intersection(z_arm,z_pk, p);
%     t_pan= get_t(p_intersection,rB,rC); %will fail when it is vertical
%     t_pancake=get_t(p_intersection,rA_pk,rB_pk);
    if isnan(p_int(1))
        u_pk=[0, 0, 0];
        p_contact=[NaN NaN];
        return
    end
    
    %Get Vectors
    e2hat=pan_normal(z_arm,p.arm);
    e1hat=pan_parallel(z_arm,p.arm); %parallel to pan
    e_pk=pk_parallel(z_pk,p.pk);

    %Get intersection positions    
    theta= max(min(dot(-e1hat,-e_pk)/(norm(e1hat)*norm(e_pk)),1),-1); % Angle between pan and pancake
    pA=p_int-(norm(rA-p_int))*cos(theta)*e1hat;
    pB=p_int+(norm(rB-p_int))*cos(theta)*e1hat;
    
    %Get intersection velocities
%     dp=pan_velocity_interpolation(z_pan,p.arm,t_pan);
    tA=get_t(pA,pan_position(:,1),pan_position(:,2));
    dpA=pan_velocity_interpolation(z_arm,p.arm,tA);
    tB=get_t(pB,pan_position(:,1),pan_position(:,2));
    dpB=pan_velocity_interpolation(z_arm,p.arm,tB);
    

    %1st constraint
    CA=dot(rA-pA,e2hat);
    CA_dot=dot(drA-dpA,e2hat);
    FcA=0;
    if CA<0
        FcA=-K_c*CA-D_c*CA_dot; %Apply a force at that point
    end
    FA=FcA*e2hat;
    tauA=cross([(rA-rC); 0],[FA; 0]);
    
    %2nd constraint
    CB=dot(rB-pB,e2hat);
    CB_dot=dot(drB-dpB,e2hat);
    FcB=0;
    if CB<0
        FcB=-K_c*CB-D_c*CB_dot; %Apply a force at that point
    end
    FB=FcB*e2hat;
    tauB=cross([(rB-rC); 0],[FB; 0]);
    
    
    F=FA+FB;
    tau=tauA(3)+tauB(3);
    
    Fx=F(1);
    Fy=F(2);
    Tau=tau;
    u_pk=[F(1) F(2) tau];
    
    
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
    
end

function [u_pk, p_contact] = discrete_impact_contact(z_arm,z_pk,p, rest_coeff, fric_coeff)
    p_contact=zeros(1,4);
    K_c=10; %10000;
    D_c=20; %.8;
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
    if tA>=0 && tA<=1
%         tA
%         T=(pA-pan_position(:,1))./(pan_position(:,2)-pan_position(:,1))
        
        CA=dot(rA-pA,e2hat);
        CA_dot=dot(drA-dpA,e2hat);
        if CA<0
            p_contact(1:2)=pA;
            FcA=-K_c*CA-D_c*CA_dot; %Apply a force at that point
        end
    end
    FA=FcA*e2hat;
    tauA=cross([(rA-rC); 0],[FA; 0]);
    
    %2nd constraint
    FcB=0;
    if tB>=0 && tB<=1
%         tB
%         pB
%         pan_position(:,1)
%         pan_position(:,2)
%         (pB-pan_position(:,1))./(pan_position(:,2)-pan_position(:,1))
        
        CB=dot(rB-pB,e2hat);
        CB_dot=dot(drB-dpB,e2hat);
        if CB<0
            p_contact(3:4)=pB;
            FcB=-K_c*CB-D_c*CB_dot; %Apply a force at that point
        end
    end
    FB=FcB*e2hat;
    tauB=cross([(rB-rC); 0],[FB; 0]);
   
    %Add Forces
    F=FA+FB;
    tau=tauA(3)+tauB(3);

    u_pk=[F(1); F(2); tau];
end

function [u_pk, p_contact] = discrete_impact_contact_dynamics(z_arm,z_pk,p, rest_coeff, fric_coeff)
    p_contact=zeros(1,4);
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