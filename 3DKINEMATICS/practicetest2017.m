%Question 2a)

clear all; 
close all; 
clc; 

syms L1 L2 L3 L4 A1 A2 A3;

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |         0 |        A1 |                               
%|  2|        L1 |      -180 |       -L2 |        A2 |                               
%|  3|         0 |       +90 |        L3 |        A3 |                                
%|  4|        L4 |         0 |         0 |         0 |


%% 
%Question 2b)

clear all; 
close all; 
clc; 

a1 = 0;
a2 = 0.5;
a3 = 0;
a4 = 0.5;

alpha1 = 0;
alpha2 = (-180)*(2*pi/360);
alpha3 = (+90)*(2*pi/360);
alpha4 = 0;

d1 = 0;
d2 = -0.25;
d3 = 0.25;
d4 = 0;

theta1 = pi/4;
theta2 = pi/3;
theta3 = pi/3;
theta4 = 0;

T0_1 = [ cos(theta1), -sin(theta1), 0, a1;
        sin(theta1)*cos(alpha1), cos(theta1)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d1;
        sin(theta1)*sin(alpha1), cos(theta1)*sin(alpha1), cos(alpha1), cos(alpha1)*d1;
        0, 0, 0, 1 ];

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

T0_4 = T0_1*T1_2*T2_3*T3_4;

T0_4 = simplify(T0_4);

%% 
%Question 2c)

%  TO FIND THE ANGLE BETWEEN TWO AXES, ONE IN THE BASE FRAME AND ONE IN THE
%  JOINT FRAME, WE USE THE DOT PRODUCT FORM OF THE ROTATION MATRIX

% For the x-axis of the base frame, and the z-axis of the 3rd joint frame,
% it is Z_j dot X_i. Steps: 
% (1) find where Z_j dot X_i is in T0_4 = (1, 3)
% (2) acos(T0_4(1, 3))

% (Can also use inspection)

angle1 = acos(T0_4(1, 3)); %OR
angle2 = 90 + (theta1 - theta2); 


