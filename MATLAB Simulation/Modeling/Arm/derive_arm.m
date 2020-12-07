name = 'arm';


% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t x dx ddx th1 dth1 ddth1 th2 dth2 ddth2 c1 c2 pA pan_length l1 l2 m1 m2 I1 I2 Ir1 Ir2 N1 N2 k kappa l0 th1_0 th2_0 g F tau1 tau2 t_param real

% Group them
q   = [th1; th2];      % generalized coordinates
dq  = [dth1; dth2];    % first time derivatives
ddq = [ddth1; ddth2];  % second time derivatives
u   = [tau1; tau2];     % controls
p   = [c1; c2; l1; l2; m1; m2; I1; I2; Ir1; Ir2; N1; N2; k; kappa; g; pA];        % parameters

% Generate Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
e1hat =  sin(th1)*ihat -cos(th1)*jhat; %Vector pointing in the direction of link1
e2hat =  sin(th1+th2)*ihat-cos(th1+th2)*jhat; %VVector pointing in the direction of link2
e2normhat = cos(th1+th2)*ihat+sin(th1+th2)*jhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

pan_length=l2-pA;
%Get key points forward kinematics
rc1= p(1)*e1hat; %position vector to the CoM of link 1
rB = p(3)*e1hat; %position vector to point B
rc2= rB+p(2)*e2hat; %position vector to the CoM of link 2
rC = rB+p(4)*e2hat; %position vector to point C (end effector)
rpA= rB+pA*e2hat; %left edge of pan
rpcom= rB+(pA+pan_length/2)*e2hat; %center of mass of pan


rt_param= rpA+t_param*pan_length*e2hat;

%time derivatives of position vectors
drc1= ddt(rc1); 
drB = ddt(rB);
drc2= ddt(rc2);
drC = ddt(rC);
dpA = ddt(rpA);

drt_param = ddt(rt_param);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

T1 = (1/2)*p(5)*dot(drc1, drc1) + (1/2)* p(7) * dth1^2;
T2 = (1/2)*p(6)*dot(drc2, drc2) + (1/2)* p(8) * dth2^2;
% Include Rotor inertias
T1r = (1/2)*Ir1*(N1*dth1)^2;
T2r = (1/2)*Ir2*(dth1 + N2*dth2)^2;

Vg1 = p(5)*g*dot(rc1,jhat);
Vg2 = p(6)*g*dot(rc2, jhat);
Ve1 = 0; %1/2*k*(sqrt(simplify(dot(rC,rC)))-l0)^2;
Ve2 = 0; %1/2*kappa*(th - th0)^2;

T = T1+T2 + T1r +T2r;%simplify(T1 + T2)
V = Vg1 + Vg2; % + Ve1 + Ve2;
Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2 = M2Q(tau2*khat,dth2*khat);
%Q_F = F2Q(F*jhat,rC);
Q = Q_tau1 + Q_tau2;%Q_F;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rc1(1:2) rB(1:2) rc2(1:2) rpA(1:2) rC(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+V;
L = T-V;
g = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;


% Rearrange Equations of Motion
A = jacobian(g,ddq)
A(1,1)
A(1,2)
A(2,1)
A(2,2)
b = A*ddq - g;

J=jacobian(rC,q);
J=J(1:2,:);
J_pan=jacobian(rpcom,q);
J_pan=J_pan(1:2,:)
[rB(1:2) rC(1:2) rc2(1:2)];
pan_position=[rpA(1:2) rC(1:2) rpcom(1:2)];

%Operational Space Mass Matrix moment inertia

% Write Energy Function and Equations of Motion
z  = [q ; dq];
directory = 'AutoDerived/Arm/';
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
matlabFunction(J,'file',[directory 'J_' name],'vars',{z p});
matlabFunction(J_pan,'file',[directory 'J_pan_' name],'vars',{z p});
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
matlabFunction(T,'file',[directory 'kinetic_energy_' name],'vars',{z p});
matlabFunction(V,'file',[directory 'potential_energy_' name],'vars',{z p});
matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});
matlabFunction(pan_position,'file',[directory 'get_pan_position'],'vars',{z p});
matlabFunction(e2hat(1:2),'file',[directory 'pan_parallel'],'vars',{z p});
matlabFunction(e2normhat(1:2),'file',[directory 'pan_normal'],'vars',{z p});
matlabFunction(rt_param(1:2),'file',[directory 'pan_position_interpolation'],'vars',{z p t_param});
matlabFunction(drt_param(1:2),'file',[directory 'pan_velocity_interpolation'],'vars',{z p t_param});
% matlabFunction(rt_param,'file',[directory 'pan_position_interpolation','vars',{z p t_param});
% matlabFunction(drt_param,'file',[directory 'pan_velocity_interpolation','vars',{z p t_param});
