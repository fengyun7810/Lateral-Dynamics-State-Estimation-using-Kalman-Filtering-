function y_n = Vehicle_measure_eq(x,param)
% ADDME Measurement function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.

global lf lr mass Cf Cr

alpha12 = (atan((x(2,:)+x(3,:)*lf)./x(1)))-param.delta;
alpha34 = atan((x(2,:)-x(3,:)*lr)./x(1));

F12= -Cf *alpha12;
F34= -Cr *alpha34;

ay = (1/mass)*(F34+F12.*cos(param.delta));
y_n = [x(1,:); ay; x(3,:)];
