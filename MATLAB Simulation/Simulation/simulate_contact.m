function [u_pk, p_contact]=simulate_contact(z_arm,z_pk, p)
% Takes in current state of arm-pancake system
% Returns Generalized Forces [Fx Fy Tau] to apply to the pancake
% It might need to change velocity? Don't remember

Fx=0;
Fy=0;
Tau=0;

[p_contact, t_pan t_pancake]= find_intersection(z_arm,z_pk, p);

u_pk=[Fx; Fy; Tau];
end

function [p_intersection, t_pan t_pancake]= find_intersection(z_arm,z_pk, p)
p_intersection=[NaN; NaN];
%Forward Kinematics
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

% From https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
p=rA_pk;
r=rB_pk-rA_pk;

q=rB;
s=rC-rB;
qp=q-p;
% c_qps=cross((q-p),s);
c_qps=qp(1)*s(2)-qp(2)*s(1);
% c_qpr=cross((q-p),r);
c_qpr=qp(1)*r(2)-qp(2)*r(1);
% c_rs=cross(r,s);
c_rs= r(1)*s(2)-r(2)*s(1);
t=c_qps/c_rs;
u=c_qpr/c_rs;

tol=1e-4;
if abs(c_rs)<tol 
    if abs(c_qpr)<tol
        %collinear (no moment)
        %Check end points
    else
        %Two lines are parallel and not intersecting
    end
else
    if t>=0 && t<=1 && u>=0 && u<=1
        %Line is intersecting and notn parallel
        %Line meets a th the point p+tr = q +us
        p_intersection=p+t.*r;
        
    else
        %Not parallel and not intersecting
    end
    
end

t_pancake=u;
t_pan=t;
end