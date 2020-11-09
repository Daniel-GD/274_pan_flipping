function E = energy_pancake(in1,in2)
%ENERGY_PANCAKE
%    E = ENERGY_PANCAKE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    09-Nov-2020 12:07:42

I = in2(3,:);
dth = in1(6,:);
dx = in1(4,:);
dy = in1(5,:);
g = in2(4,:);
m = in2(2,:);
y = in1(2,:);
E = (I.*dth.^2)./2.0+(m.*(dx.^2+dy.^2))./2.0+g.*m.*y;
