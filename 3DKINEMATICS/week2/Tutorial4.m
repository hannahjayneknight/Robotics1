%% Tutorial 4:
clear all;

%% Question 2

Rz = [0.866 -0.5 0;
      0.5 0.866 0;
      0 0 1];
 
Ry = [0.7071 0 0.7071;
      0 1 0;
      -0.7071 0 0.7071];

Rx = [1 0 0;
      0 0 -1;
      0 1 0];

R = Rz*Ry*Rx;

%% Question 7
clear all;
syms P1 P2 P3 R1 R2 R3;

T1 = [R1 P1;
      0 1];
T2 = [R2 P2;
      0 1];
T3 = [R3 P3;
      0 1];

T4 = simplify(T1*T2*T3);

%% Question 8
e4 = 1/2 * sqrt(1 + 0.866 + 0.433 + 0.5);
e1 = (0.7+0.866)/(4*e4);
e2 = (0-0.43)/(4*e4);
e3 = (0.25+0.5)/(4*e4);


%% Question 9
clear all;
syms theta1 theta2 L1 L2;

T = [L1*cos(theta1)+L2*cos(theta1+theta2);
     L1*sin(theta1)+L2*sin(theta1+theta2);
     0];
 
J = jacobian(T,[theta1,theta2, theta3])

%% Question 10
force = [1; 1; 1];
torque = transpose(J)*force







