function x_n = Vehicle_state_eq(x,param)
% ADDME Dynamic model function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.

global lf lr mass Iz Cf Cr

alpha12 = (atan((x(2,:)+x(3,:)*lf)./x(1)))-param.delta;
alpha34 = atan((x(2,:)-x(3,:)*lr)./x(1));

F12= -Cf *alpha12;
F34= -Cr *alpha34;

vxx = 1/mass*(-F12*sin(param.delta))+x(3)*x(2);
vyy = 1/mass*(F34+F12*cos(param.delta));
yaww = 1/Iz*(lf*F12*cos(param.delta)-lr*F34);

%f_x = [sin(param.delta)*Cf*((x(2)-x(3)*lr)/x(1))/mass+x(3)*x(2)
%-(Cr*(x(2)-x(3)*lr)/x(1)+cos(param.delta)*Cf*((x(2)+x(3)*lf)/x(1)-param.delta))/mass+x(3)*x(2)
%(-lf*cos(param.delta)*Cf*((x(2)+x(3)*lf)/x(1)-param.delta)+lr*Cr*(x(2)-x(3)*lr)/x(1))/Iz];

f_x = [vxx; vyy; yaww];

% Integrate using Runge Kutta (in the script folder) or simple euler forward

f = @(x)[f_x(1,:);f_x(2,:);f_x(3,:)];
x_n = rk4(f,param.dt,x(1:3,:));

%%end
