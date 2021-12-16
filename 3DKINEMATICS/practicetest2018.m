%Question 2a)

clear all; 
close all; 
clc; 

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |         0 |    theta1 |                               
%|  2|        L1 |         0 |         0 |    theta2 |                               
%|  3|        L2 |         0 |         0 |         0 |                                


%% 
%Question 2b)

clear all; 
close all; 
clc; 

syms theta1 theta2 L1 L2;

a1 = 0;
a2 = L1;
a3 = L2;

alpha1 = 0;
alpha2 = 0;
alpha3 = 0;

d1 = 0;
d2 = 0;
d3 = 0;

theta3 = 0;

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

T0_4 = T0_1*T1_2*T2_3;

T0_4 = simplify(T0_4);

%% 
%Question 2c)

C_p = [ 0.5;
        -0.6;
        -1; 
        1; ];
R_p = simplify(T0_4*C_p);

answer = subs(R_p, {theta1, theta2, L1, L2}, {(60)*(2*pi/360), (30)*(2*pi/360), 0.75, 0.5});


%% 
%Question 2d)

% SEE WORKING OUT IN ONENOTE!
