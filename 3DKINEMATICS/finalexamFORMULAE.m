% FINAL EXAM FORMULAE

%% 
%Transformation matrix 
% replace i with numbers according to which transformation, then multiply 
% all together to get the homogenous matrix

syms thetai alphai di ai;

%CONVERTING DEGS TO RADS = (angle)*(2*pi/360)

%CONVERTING RADS TO DEGS = (angle)*(360/2*pi)

a1 = 0;
a2 = 0;
a3 = 0;
a4 = 0;

alpha1 = 0;
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;

d1 = 0;
d2 = 0;
d3 = 0;
d4 = 0;

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;

Tj_i = [ cos(thetai), -sin(thetai), 0, ai;
        sin(thetai)*cos(alphai), cos(thetai)*cos(alphai), -sin(alphai), -sin(alphai)*di;
        sin(thetai)*sin(alphai), cos(thetai)*sin(alphai), cos(alphai), cos(alphai)*di;
        0, 0, 0, 1 ];

%T0_4 = T0_1*T1_2*T2_3*T3_4;

%% 
%Jacobian stuff