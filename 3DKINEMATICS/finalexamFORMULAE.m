% FINAL EXAM FORMULAE

%% 
%Transformation matrix 
% replace i with numbers according to which transformation, then multiply 
% all together to get the homogenous matrix

syms thetai alphai di ai;

syms theta1 alpha1 d1 a1 theta2 alpha2 d2 a2 theta3 alpha3 d3 a3 theta4 alpha4 d4 a4 L1 L2 L3 L4;

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

%T0_4 = simplify(T0_4);

%% 

%  TO FIND THE ANGLE BETWEEN TWO AXES, ONE IN THE BASE FRAME AND ONE IN THE
%  JOINT FRAME, WE USE THE DOT PRODUCT FORM OF THE ROTATION MATRIX

% For the x-axis of the base frame, and the z-axis of the 3rd joint frame,
% it is Z_j dot X_i. Steps: 
% (1) find where Z_j dot X_i is in T0_4 = (1, 3)
% (2) acos(T0_4(1, 3))

% (Can also use inspection)

angle1 = acos(T0_4(1, 3)); %OR
angle2 = 90 + (theta1 - theta2); 

%%
% When using pi write sym(pi) to avoid floating point error


%%
%Jacobian stuff

syms J f pdot;

torque = J.'*f;

inverse = inv(J); %when J square
thetadot = inverse*pdot;
pseudoinverse = pinv(J); %when J not square
thetadot2 = pseudoinverse*pdot;

%%
%Quaternion stuff

syms e1 e2 e3 e4;

e1 = 0;
e2 = 0;
e3 = 0;
e4 = 0;

Re = [  1-2*(e2^2)-2*(e3^2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4);
        2*(e1*e2+e3*e4), 1-2*(e1^2)-2*(e3^2), 2*(e2*e3-e1*e4);
        2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1^2)-2*(e2^2)];

T = [   Re, p;
        0, 0, 0, 1 ];


%%
%point vector

p = [   x;
        y;
        z;
        1   ];

p2 = [   x;
        y;
        z   ];




