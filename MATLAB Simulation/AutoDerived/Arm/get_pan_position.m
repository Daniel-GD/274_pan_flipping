function out1 = get_pan_position(in1,in2)
%GET_PAN_POSITION
%    OUT1 = GET_PAN_POSITION(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    15-Nov-2020 16:56:29

c2 = in2(2,:);
l1 = in2(3,:);
l2 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l1.*t2;
t6 = cos(t4);
t7 = l1.*t3;
t8 = sin(t4);
t9 = -t5;
out1 = reshape([t7,t9,t7+l2.*t8,t9-l2.*t6,t7+c2.*t8,t9-c2.*t6],[2,3]);