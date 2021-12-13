%Question 1

%+---+-----------+-----------+-----------+-----------+                               
%| i |     ai    |    alphai |        di |    thetai |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |         0 |        L1 |    theta1 |                               
%|  2|         0 |     -pi/4 |         0 |    theta2 |                               
%|  3|        L2 |         0 |         0 |    theta3 |                                
%|  4|        L3 |         0 |         0 |         0 |                               
 

%% 
%Question 2

theta2 = -pi/3;
alpha2 = -pi/4;
L2 = 0.25;
theta3 = pi/2;
alpha3 = 0;
L3 = 1;

T0_1 = [cos(pi/4), -sin(pi/4), 0, 0;
        sin(pi/4)*cos(0), cos(pi/4)*cos(0), -sin(0), -sin(0)*0.5;
        sin(pi/4)*sin(0), cos(pi/4)*sin(0), cos(0), cos(0)*0.5;
        0, 0, 0, 1 ];

T1_2 = [cos(theta2), -sin(theta2), 0, 0;
        sin(theta2)*cos(alpha2), cos(theta2)*cos(alpha2), -sin(alpha2), -sin(alpha2)*L2;
        sin(theta2)*sin(alpha2), cos(theta2)*sin(alpha2), cos(alpha2), cos(alpha2)*L2;
        0, 0, 0, 1 ];

T2_3 = [cos(theta3), -sin(theta3), 0, 0;
        sin(theta3)*cos(alpha3), cos(theta3)*cos(alpha3), -sin(alpha3), -sin(alpha3)*L3;
        sin(theta3)*sin(alpha3), cos(theta3)*sin(alpha3), cos(alpha3), cos(alpha3)*L3;
        0, 0, 0, 1 ];

T3_4 = [cos(theta3), -sin(theta3), 0, 0;
        sin(theta3)*cos(alpha3), cos(theta3)*cos(alpha3), -sin(alpha3), -sin(alpha3)*L3;
        sin(theta3)*sin(alpha3), cos(theta3)*sin(alpha3), cos(alpha3), cos(alpha3)*L3;
        0, 0, 0, 1 ];

T0_3 = T0_1*T1_2*T2_3;


%% 
%Question 3


%% 
%Question 4


%% 
%Question 5


%% 
%Question 6


%% 
%Question 7


%% 
%Question 8


%% 
%Question 9


%% 
%Question 10



%%

% My choice is 3

%—– Explanation with a Matlab code——-
% Please run the following Matlab code with commented lines to verify my answer
clear all; %Clear all variables
close all; %Close all figures
clc; %Clear screen
syms l1 l2 l3 theta1 theta2 theta3 real; %Define symbolic variables as real variables
Ttip = [l1*cos(theta1) + l2*sin(theta1) + l3*cos(theta1)*cos(theta2);...
l1*sin(theta1) - l2*cos(theta1) + l3*sin(theta1)*cos(theta2);...
l3*sin(theta2)];%The position vector at the tip of the robot
J = jacobian(Ttip,[theta1,theta2 theta3]); %Jacobian matrix
f = [1;2;3] %force vector
% Therefore, ......your arguments
%%

%—– If there is no need for a Matlab code to explain your choice, you can just give text in
commented lines like...
% A property of a homogeneous transformation matrix is ABC.
% Continue explanations in commented lines....
% ....Therefore, the correct choice is 3.
% —————————-