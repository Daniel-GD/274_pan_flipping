function A = A_pancake(in1,in2)
%A_PANCAKE
%    A = A_PANCAKE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Oct-2020 01:37:10

I = in2(3,:);
m = in2(2,:);
A = reshape([m,0.0,0.0,0.0,m,0.0,0.0,0.0,I],[3,3]);