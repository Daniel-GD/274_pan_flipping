function out1 = pk_parallel(in1,in2)
%PK_PARALLEL
%    OUT1 = PK_PARALLEL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    09-Nov-2020 12:07:43

th = in1(3,:);
out1 = [cos(th);sin(th)];
