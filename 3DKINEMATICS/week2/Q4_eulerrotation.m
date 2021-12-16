clear all; syms alpha beta gamma real;
%The end effector of a robot manipulator has roll = 90◦, yaw = 60◦, and 
% pitch = 15◦. What is the closest x-y-z Euler rotation matrix?


% Roll - gamma (rotation around the x axis) 
% Yaw - alpha (rotation around the y axis) 
%Pitch - beta (rotation around the z axis)
Rz = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
Rx = [1 0 0; 0 cos(gamma) -sin(gamma); 0 sin(gamma) cos(gamma)];
% For x-y-z Euler rotation 
R = simplify(Rz*Ry*Rx)
gamma = pi/2; alpha = pi/3; beta = pi/12;
vpa(subs(R))
