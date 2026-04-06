function [y, t] = aerobalancin_ode(x)

t = 0:0.1:10;

[t, z] = ode45(@(t,z) odemodel(z, x(1), x(2), x(3), x(4), x(5), x(6), x(7)), t, [0; 0]);

y = z(:,1)';
t = t';

end

function dz = odemodel(z, mb, m1, m2, l1, l2, g, c)

L = l1 + l2;
d = (l1 + l2)/2 - l2;
m = mb + m1 + m2;
J = (1/12)*mb*L^2 + mb*d^2 + m1*l1^2 + m2*l2^2;

T1 = 1;
T2 = 1;

theta     = atan2(sin(z(1)), cos(z(1)));
theta_dot = z(2);

theta_ddot = (T1*l1 - T2*l2 - m*g*d*sin(pi/2 - theta) - c*theta_dot) / J;

dz = [theta_dot; theta_ddot];

end