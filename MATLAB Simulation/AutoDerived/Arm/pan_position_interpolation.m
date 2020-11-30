function out1 = pan_position_interpolation(in1,in2,t_param)
%PAN_POSITION_INTERPOLATION
%    OUT1 = PAN_POSITION_INTERPOLATION(IN1,IN2,T_PARAM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    17-Nov-2020 19:29:58

l1 = in2(3,:);
l2 = in2(4,:);
pA = in2(16,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = th1+th2;
t5 = -pA;
t3 = cos(t2);
t4 = sin(t2);
t6 = l2+t5;
out1 = [pA.*t4+l1.*sin(th1)+t4.*t6.*t_param;t3.*t5-l1.*cos(th1)-t3.*t6.*t_param];
