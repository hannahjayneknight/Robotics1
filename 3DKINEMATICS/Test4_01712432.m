%Question 1

%{

answer = (2) 

reason = The vector has not been enlarged, only rotated, therefore the
magnitude of the vector remains the same.

%}

%% 
%Question 2

%{

answer = (2) YES

reason = X-Z-X is a valid Euler rotation. This is the same as writing
X1-Z-X2 as the first x-axis has moved.

The objective of the first two Euler rotations with moving frames is to get one axis to coincide
with the corresponding axis of the target frame. Then, since the three axes are orthogonal, one
final rotation around the axis that coincides with the target axis will make all three axes to
coincide.

%}

%% 
%Question 3

%{

answer = (1) NO

reason = Leonard Euler said - ”Any rotation can be described by three successive rotations around
linearly independent axes”.

No because v is not linearly independent of the first two axes of rotation.

%}
%% 
%Question 4

%{

answer = (3)

reason = When a Jacobian is singular, it means that two columns in the
matrix are parallel. This means that (4) and (2) are conditions for
singularity since they are descibing parallel columns. Collinearity means
two vectors are parallel (they lie on the same line) therefore (1) is a
condition for singularity which leaves (3) as the correct answer to the
question.

%}


%% 
%Question 5

clear all; 
close all; 
clc; 

f = [   10;
         5;
         20;    ];
torque = [  30, 15, 10];

J = f*torque;


%% 
%Question 6
clear all; 
close all; 
clc; 

angle = acos(0.591)*180/pi;

%{

answer = (3)

reason = Angle between x-axis of B and z-axis of A is given by position (3,
1) of the rotation matrix which is x_B dot z_A. Multiplying by 180/ pi
gives this value in degrees.

%}


%% 
%Question 7

clear all; 
close all; 
clc; 

syms theta1 alpha1 d1 a1 theta2 alpha2 d2 a2 theta3 alpha3 d3 a3 L1 L2 L3;

a1 = L1;

alpha1 = sym(pi/4);

d1 = 0;

T0_1 = [ cos(theta1), -sin(theta1), 0, a1;
        sin(theta1)*cos(alpha1), cos(theta1)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d1;
        sin(theta1)*sin(alpha1), cos(theta1)*sin(alpha1), cos(alpha1), cos(alpha1)*d1;
        0, 0, 0, 1 ];

%{

answer = (3)

reason = See above code.

%}

%% 
%Question 8

%{

answer = (4)

reason = The translation vector from frame i to i-1 is:

P = [   ai;
        -sin(alphai*di);
        cos(alphai*di);
        1;      ];

The angle between P and the xi-1 axis is given by solution (4).

%}


%% 
%Question 9

clear all; 
close all; 
clc; 

R = [   -0.224, -0.836, 0.5;
        -0.774, -0.158, -0.612;
        0.591, -0.524, -0.612;
    ];

epsilon4 = 0.5*sqrt(1 + R(1,1) + R(2,2) + R(3,3));
epsilon1 = (R(3,2)-R(2,3))/(4*epsilon4);
epsilon2 = (R(1,3)-R(3,1))/(4*epsilon4);
epsilon3 = (R(2,1)-R(1,2))/(4*epsilon4);

c1 = epsilon1*sin((pi/6)/2);
c2 = epsilon2*sin((pi/6)/2);
c3 = epsilon3*sin((pi/6)/2);


%{

answer = (4)

reason = When running the above code, none of solutions contained the
correct answer.

%}


%% 
%Question 10


clear all; 
close all; 
clc; 

syms theta1 alpha1 d1 a1 theta2 alpha2 d2 a2 theta3 alpha3 d3 a3 L1 L2 L3;

a1 = L1;
a2 = 0;
a3 = L3;

alpha1 = sym(pi/4);
alpha2 = sym(pi/2);
alpha3 = 0;

d1 = 0;
d2 = L2;
d3 = 0;

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

T0_3 = simplify(T0_1*T1_2*T2_3);

p = T0_3(1:3,end);

J = simplify(jacobian(p, [theta1,theta2, theta3]));

%{

answer = (3)

reason = If we find the transformation matrix from frame 1 to 3, the last
column of this is the position vector of the origin of frame 3. We can find
the Jacobian of this to find the Jacobian matrix of the origin of frame 3.

See code above for final solution.

%}

