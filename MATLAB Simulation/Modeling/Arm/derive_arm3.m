name = 'arm3';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t x dx ddx th1 dth1 ddth1 th2 dth2 ddth2 th3 dth3 ddth3 c1 c2 c3 l1 l2 l3 m1 m2 m3 I1 I2 I3 k kappa l0 th1_0 th2_0 th3_0 g F tau1 tau2 tau3 real

% Group them
q   = [th1; th2; th3];      % generalized coordinates
dq  = [dth1; dth2; dth3];    % first time derivatives
ddq = [ddth1; ddth2; ddth3];  % second time derivatives
u   = [tau1; tau2; tau3];     % controls
p   = [c1; c2; c3; l1; l2; l3; m1; m2; m3; I1; I2; I3; k; kappa; th1_0; th2_0; th3_0; g];        % parameters

% Generate Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
e1hat =  sin(th1)*ihat -cos(th1)*jhat; %Vector pointing in the direction of link1
e2hat =  sin(th1+th2)*ihat-cos(th1+th2)*jhat; %Vector pointing in the direction of link2
e3hat =  sin(th1+th2+th3)*ihat-cos(th1+th2+th3)*jhat; %Vector pointing in the direction of link3
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

%Get key points forward kinematics
rc1= p(1)*e1hat; %position vector to the CoM of link 1
rB = p(4)*e1hat; %position vector to point B
rc2= rB+p(2)*e2hat; %position vector to the CoM of link 2
rC = rB+p(5)*e2hat; %position vector to point C (end effector)
rc3= rC+p(3)*e3hat; %position vector to the CoM of link 2
rD = rC+p(6)*e3hat; %position vector to point C (end effector)


%time derivatives of position vectors
drc1= ddt(rc1); 
drB = ddt(rB);
drc2= ddt(rc2);
drC = ddt(rC);
drc3= ddt(rc3);
drD = ddt(rD);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

T1 = (1/2)*p(7)*dot(drc1, drc1) + (1/2)* p(10) * dth1^2;
T2 = (1/2)*p(8)*dot(drc2, drc2) + (1/2)* p(11) * dth2^2;
T3 = (1/2)*p(9)*dot(drc3, drc3) + (1/2)* p(12) * dth3^2;

Vg1 = p(7)*g*dot(rc1,jhat);
Vg2 = p(8)*g*dot(rc2, jhat);
Vg3 = p(9)*g*dot(rc3, jhat);
Ve1 = 0; %1/2*k*(sqrt(simplify(dot(rC,rC)))-l0)^2;
Ve2 = 0; %1/2*kappa*(th - th0)^2;
Ve3 = 1/2*kappa*(th3 - th3_0)^2; %1/2*kappa*(th - th0)^2;

T = T1+T2+T3;%simplify(T1 + T2 + T3)
V = Vg1 + Vg2 + Vg3 + Ve3; % + Ve1 + Ve2 + Ve3;
Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2 = M2Q(tau2*khat,dth2*khat);
Q_tau3 = M2Q(tau3*khat,dth3*khat);
%Q_F = F2Q(F*jhat,rC);
Q = Q_tau1 + Q_tau2 + Q_tau3;%Q_F;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rc1(1:2) rB(1:2) rc2(1:2) rC(1:2) rc3(1:2) rD(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+V;
L = T-V;
g = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;


% Rearrange Equations of Motion
A = jacobian(g,ddq);
b = A*ddq - g;

% Write Energy Function and Equations of Motion
z  = [q ; dq];
directory = '../../AutoDerived/Arm/';
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
matlabFunction(T,'file',[directory 'kinetic_energy_' name],'vars',{z p});
matlabFunction(V,'file',[directory 'potential_energy_' name],'vars',{z p});
matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});
