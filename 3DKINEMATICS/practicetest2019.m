%Question 2a)

clear all; 
close all; 
clc; 

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |         0 |    theta1 |                               
%|  2|         0 |     -pi/4 |         0 |    theta2 |                               
%|  3|        L1 |         0 |         0 |    theta3 | 
%|  4|        L2 |         0 |         0 |         0 | 


%% 
%Question 2b)

% Transformation matrix from the tip to frame {A} means that using the DH
% table we are only going from i=2 to i=4

clear all; 
close all; 
clc; 

syms theta1 theta2 theta3 L1 L2;

a1 = 0;
a2 = 0;
a3 = L1;
a4 = L2;

alpha1 = 0;
alpha2 = sym(-pi/2);
alpha3 = 0;
alpha4 = 0;

d1 = 0;
d2 = 0;
d3 = 0;
d4 = 0;

theta4 = 0;

T1_2 = [ cos(theta2), -sin(theta2), 0, a2;
        sin(theta2)*cos(alpha2), cos(theta2)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d2;
        sin(theta2)*sin(alpha2), cos(theta2)*sin(alpha2), cos(alpha2), cos(alpha2)*d2;
        0, 0, 0, 1 ];

T2_3 = [ cos(theta3), -sin(theta3), 0, a3;
        sin(theta3)*cos(alpha3), cos(theta3)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d3;
        sin(theta3)*sin(alpha3), cos(theta3)*sin(alpha3), cos(alpha3), cos(alpha3)*d3;
        0, 0, 0, 1 ];

T3_4 = [ cos(theta4), -sin(theta4), 0, a4;
        sin(theta4)*cos(alpha4), cos(theta4)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d4;
        sin(theta4)*sin(alpha4), cos(theta4)*sin(alpha4), cos(alpha4), cos(alpha4)*d4;
        0, 0, 0, 1 ];

T1_4 = T1_2*T2_3*T3_4;

T1_4 = simplify(T1_4);

%% 
%Question 2c)

clear all; 
close all; 
clc;

syms theta1 theta2 theta3 L1 L2 m g;

J = [   -L1*sin(theta2) - L2*sin(theta2+theta3), -L2*sin(theta2+theta3);
        0, 0;
        -L1*cos(theta2)-L2*cos(theta2+theta3), -L2*cos(theta2+theta3)];

f = [   0; 
        0; 
        -m*g];

torque = J.'*f;
torque = simplify(torque); 
%=> Ta = g*m*(L2*cos(theta2 + theta3) + L1*cos(theta2))
%=> Tb = L2*g*m*cos(theta2 + theta3)

%joint configuration when torque around B axis is 0:
%=> Tb = L2*g*m*cos(theta2 + theta3) = 0
%=> theta2 + theta3 = pi/2
%=> one possible configuration is theta 2 = pi/4 and theta 3 = pi/4 
% although other configurations exist

%% 
%Question 2d)

theta2 = 0;
pseudoinverse = pinv(J); %J not square so we use the pseudo-inverse
