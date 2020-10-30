function [u_pk]=simulate_contact(z_arm,z_pk, p)
% Takes in current state of arm-pancake system
% Returns Generalized Forces [Fx Fy Tau] to apply to the pancake
% It might need to change velocity? Don't remember

Fx=0;
Fy=0;
Tau=0;

u_pk=[Fx; Fy; Tau];
end
function [p, x_pancake]= find_intersection(z_arm,z_pk, p)
%Forward Kinematics
arm_keypoints= keypoints_arm(z_arm,p);
pk_keypoints= keypoints_pancake(z_pk,p);
%Get arm keypoints
rc1 = arm_keypoints(:,1); % position vector to the CoM of link 1
rB = arm_keypoints(:,2); %position vector to point B
rc2 = arm_keypoints(:,3); % position vector to the CoM of link 2
rC = arm_keypoints(:,4); %position vector to point C

%Get pancake keypoints 
rc_pk = pk_keypoints(:,1); % position vector to the CoM of pancake
rA_pk = pk_keypoints(:,2); %position vector to point A in pancake
rB_pk = pk_keypoints(:,3); % position vector to point B in pancake
p=[0;0];
x_pancake=0;
end