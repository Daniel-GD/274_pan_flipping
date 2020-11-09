name = 'pancake';
directory = '../../AutoDerived/Pancake/';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
% syms t x dx ddx th1 dth1 ddth1 th2 dth2 ddth2 c1 c2 l1 l2 m1 m2 I1 I2 k kappa l0 th1_0 th2_0 g F tau1 tau2 real
syms t x dx ddx y dy ddy th dth ddth x0 y0 th0 l m I g Fx Fy Tau t_param real
% Group them
q   = [x; y; th];      % generalized coordinates (theta is respect to horizontal
dq  = [dx; dy; dth];    % first time derivatives
ddq = [ddx; ddy; ddth];  % second time derivatives
u   = [Fx; Fy; Tau];     % controls
% p   = [c1; c2; l1; l2; m1; m2; I1; I2; k; kappa; th1_0; th2_0; g];        % parameters
p=[l; m; I; x0; y0; th0; g];

% Generate Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
e1hat =  cos(th)*ihat +sin(th)*jhat; %Vector pointing in the direction of the flat pancake
% e1norm = 

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

%Get key points forward kinematics
rc=[x; y; 0]; %Position of CoM
rA=rc-l/2*e1hat; %1st end point of pancake
rB=rc+l/2*e1hat; %2nd end point of pancake

rt_param= rA+ l*t_param*e1hat;
% rc1= p(1)*e1hat; %position vector to the CoM of link 1
% rB = p(3)*e1hat; %position vector to point B
% rc2= rB+p(2)*e2hat; %position vector to the CoM of link 2
% rC = rB+p(4)*e2hat; %position vector to point C (end effector)

%time derivatives of position vectors
drc= ddt(rc); 
drA = ddt(rA); %they are the same as rc
drB = ddt(rB);

drt_param=ddt(rt_param);
% drc2= ddt(rc2);
% drC = ddt(rC);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

T1 = (1/2)*m*dot(drc, drc) + (1/2)* I * dth^2;
% T2 = (1/2)*p(6)*dot(drc2, drc2) + (1/2)* p(8) * dth2^2;

Vg1 = m*g*dot(rc,jhat);
% Vg2 = p(6)*g*dot(rc2, jhat);
% Ve1 = 0; %1/2*k*(sqrt(simplify(dot(rC,rC)))-l0)^2;
% Ve2 = 0; %1/2*kappa*(th - th0)^2;

T = T1;%simplify(T1 + T2)
V = Vg1; % + Ve1 + Ve2;
% Q_tau1 = M2Q(tau1*khat,dth1*khat);
% Q_tau2 = M2Q(tau2*khat,dth2*khat);
Q_Fx=F2Q(Fx*ihat,rc);
Q_Fy=F2Q(Fy*jhat,rc);
Q_tau = M2Q(Tau*khat,dth*khat);
%Q_F = F2Q(F*jhat,rC);
Q = Q_Fx+Q_Fy+Q_tau;%Q_tau1 + Q_tau2;%Q_F;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rc(1:2) rA(1:2) rB(1:2)];

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

JA = jacobian(rA,q);
JB = jacobian(rB,q);
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
matlabFunction(T,'file',[directory 'kinetic_energy_' name],'vars',{z p});
matlabFunction(V,'file',[directory 'potential_energy_' name],'vars',{z p});
matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});
matlabFunction(e1hat(1:2),'file',[directory 'pk_parallel'],'vars',{z p});
% matlabFunction(e2normhat(1:2),'file',[directory 'pk_normal'],'vars',{z p});

matlabFunction([rA(1:2) rB(1:2)],'file',[directory 'get_pancake_position' ],'vars',{z p});
matlabFunction([drA(1:2) drB(1:2)],'file',[directory 'get_pancake_velocity' ],'vars',{z p});
matlabFunction(drc(1:2),'file',[directory 'get_pancake_com_velocity' ],'vars',{z p});
matlabFunction(rt_param(1:2),'file',[directory 'pk_position_interpolation'],'vars',{z p t_param});
matlabFunction(drt_param(1:2),'file',[directory 'pk_velocity_interpolation'],'vars',{z p t_param});

matlabFunction(JA,'file',[directory 'jacobian_pk_A'],'vars',{z p});
matlabFunction(JB,'file',[directory 'jacobian_pk_B'],'vars',{z p});
