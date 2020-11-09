function out1 = pk_position_interpolation(in1,in2,t_param)
%PK_POSITION_INTERPOLATION
%    OUT1 = PK_POSITION_INTERPOLATION(IN1,IN2,T_PARAM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    06-Nov-2020 14:12:42

l = in2(1,:);
th = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = cos(th);
t3 = sin(th);
out1 = [x-(l.*t2)./2.0+l.*t2.*t_param;y-(l.*t3)./2.0+l.*t3.*t_param];
