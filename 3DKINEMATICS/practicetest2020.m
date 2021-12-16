%Question 2a)

clear all; 
close all; 
clc; 

e1 = 0.3062;
e2 =  0.3062;
e3 = 0.25;
e4 = 0.866;

Re = [  1-2*(e2^2)-2*(e3^2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4);
        2*(e1*e2+e3*e4), 1-2*(e1^2)-2*(e3^2), 2*(e2*e3-e1*e4);
        2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1^2)-2*(e2^2)];

p = [   0.25;
        0;
        -2      ];

CG_T_CR = [   Re, p;
        0, 0, 0, 1 ];

%% 
%Question 2b)

%DH table

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |        L1 |    2*pi/3 |                               
%|  2|        L2 |      pi/3 |        L3 |         0 | 

a1 = 0;
a2 = 0.25;

alpha1 = 0;
alpha2 = sym(pi/3);

d1 = 0.5;
d2 = 1.5;

theta1 = sym(2*pi/3);
theta2 = 0;

T0_1 =  [ cos(theta1), -sin(theta1), 0, a1;
        sin(theta1)*cos(alpha1), cos(theta1)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d1;
        sin(theta1)*sin(alpha1), cos(theta1)*sin(alpha1), cos(alpha1), cos(alpha1)*d1;
        0, 0, 0, 1 ];

T1_2 = [ cos(theta2), -sin(theta2), 0, a2;
        sin(theta2)*cos(alpha2), cos(theta2)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d2;
        sin(theta2)*sin(alpha2), cos(theta2)*sin(alpha2), cos(alpha2), cos(alpha2)*d2;
        0, 0, 0, 1 ];

G_T_CG = simplify(T0_1*T1_2);

%% 
%Question 2c)

CR_v_B = [  -2;
            0.5;
            1;
            1   ];

c = G_T_CG * CG_T_CR * CR_v_B;
c= round(c, 2);

%% 
%Question 2d)
