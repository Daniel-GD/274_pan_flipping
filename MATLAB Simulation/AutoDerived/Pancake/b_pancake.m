function b = b_pancake(in1,in2,in3)
%B_PANCAKE
%    B = B_PANCAKE(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Oct-2020 01:25:33

Fx = in2(1,:);
Fy = in2(2,:);
Tau = in2(3,:);
g = in3(7,:);
m = in3(2,:);
b = [Fx;Fy-g.*m;Tau];
