%Question a

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |        L1 |    theta1 |                               
%|  2|         0 |     -pi/4 |         0 |    theta2 |                               
%|  3|        L2 |         0 |         0 |    theta3 |                                
%|  4|        L3 |         0 |         0 |         0 |                               
 

%% 
%Question b

clear all; 
close all; 
clc; 

theta1 = pi/4;
alpha1 = 0;
L1 = 0.5;
a1 = 0;

theta2 = -pi/3;
alpha2 = -pi/4;
L2 = 0.25;
theta3 = pi/2;
alpha3 = 0;
L3 = 1;

T0_1 = [cos(theta1), -sin(theta1), 0, 0;
        sin(theta1), cos(theta1), 0, 0;
        0, 0, 1, L1;
        0, 0, 0, 1 ];

T1_2 = [cos(theta2), -sin(theta2), 0, 0;
        0, 0, 1, 0;
        -sin(theta2), -cos(theta2), 0, 0;
        0, 0, 0, 1 ];

T2_3 = [cos(theta3), -sin(theta3), 0, L2;
        sin(theta3), cos(theta3), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1 ];

T3_4 = [1, 0, 0, L3;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1 ];

T0_4 = T0_1*T1_2*T2_3*T3_4;


%% 
%Question c

clear all; 
close all; 
clc; 

gamma_degs = acosd(-0.3536); 
gamma_rads = gamma_degs*(pi/180);
