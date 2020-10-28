function T = kinetic_energy_pancake(in1,in2)
%KINETIC_ENERGY_PANCAKE
%    T = KINETIC_ENERGY_PANCAKE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Oct-2020 01:25:33

I = in2(3,:);
dth = in1(6,:);
dx = in1(4,:);
dy = in1(5,:);
m = in2(2,:);
T = (I.*dth.^2)./2.0+(m.*(dx.^2+dy.^2))./2.0;
