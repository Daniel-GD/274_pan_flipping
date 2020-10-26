function T = kinetic_energy_doublepend(in1,in2)
%KINETIC_ENERGY_DOUBLEPEND
%    T = KINETIC_ENERGY_DOUBLEPEND(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Oct-2020 17:20:05

I1 = in2(7,:);
I2 = in2(8,:);
c1 = in2(1,:);
c2 = in2(2,:);
dth1 = in1(3,:);
dth2 = in1(4,:);
l1 = in2(3,:);
m1 = in2(5,:);
m2 = in2(6,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = c1.^2;
t6 = dth1.^2;
t7 = cos(t4);
t8 = sin(t4);
T = (m2.*((dth1.*(c2.*t7+l1.*t2)+c2.*dth2.*t7).^2+(dth1.*(c2.*t8+l1.*t3)+c2.*dth2.*t8).^2))./2.0+(I1.*t6)./2.0+(m1.*(t2.^2.*t5.*t6+t3.^2.*t5.*t6))./2.0+(I2.*dth2.^2)./2.0;
