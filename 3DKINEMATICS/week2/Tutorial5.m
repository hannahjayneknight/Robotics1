%% Tutorial 4:
clear all;

%% Question 2
J = [2 0.5 1.5;
     -0.5 1.75 0.25;
      1.5 0.5 2.5];
f = [-1.5; 2; 0.5];
 
torque = transpose(J) * f;

%% Question 4
clear all;
syms alpha beta gamma real;

Rz = [cos(alpha) -sin(alpha) 0;
      sin(alpha) cos(alpha) 0;
      0 0 1];
 
Ry = [cos(beta) 0 sin(beta);
      0 1 0;
      -sin(beta) 0 cos(beta)];

Rx = [1 0 0;
      0 cos(gamma) -sin(gamma);
      0 sin(gamma) cos(gamma)];

R = simplify(Rz*Ry*Rx);

%Test for properties of a rotation matrix
alpha = 60*2*pi/360;
beta = 15*2*pi/360;
gamma = 90*2*pi/360;

RR = eval(R);


%% Question 5:
clear all;
syms alpha theta a b;

S0 = [1 0 0 a;
      0 cos(alpha) -sin(alpha) 0;
      0 sin(alpha) cos(alpha) 0;
      0 0 0 1];
S1 = [cos(theta) 0 -sin(theta) 0;
      0 1 0 b;
      sin(theta) 0 cos(theta) 0;
      0 0 0 1];
 
R = simplify(S1*S0);


%% Question 6
clear all;
syms alpha theta a b c;

P = [a*cos(alpha) + b*cos(alpha + theta);
     a*sin(alpha) + b*sin(alpha + theta);
     c];
F = [2;
     1;
     3];
 
J = jacobian(P, [alpha, theta]);
 
torque = transpose(J)*F;
 
%% Question 8???????
clear all;
Rz =  [1 0 0;
       0 cos(pi/6) -sin(pi/6);
       0 sin(pi/6) cos(pi/6)];
Rx = [cos(pi/3) -sin(pi/3) 0;
      sin(pi/3) cos(pi/3) 0;
      0 0 1];
R = Rz*Rx
 

%%

T = [cos(theta) -sin(theta) 0 a;
     sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
     sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
     0 0 0 1];

alpha = pi/3;
theta = pi/6;
RR = eval(T)

