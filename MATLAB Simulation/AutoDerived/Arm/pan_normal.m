function out1 = pan_normal(in1,in2)
%PAN_NORMAL
%    OUT1 = PAN_NORMAL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    03-Dec-2020 14:10:51

th1 = in1(1,:);
th2 = in1(2,:);
t2 = th1+th2;
out1 = [cos(t2);sin(t2)];
