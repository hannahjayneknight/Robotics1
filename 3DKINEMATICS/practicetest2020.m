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

T = [   Re, p;
        0, 0, 0, 1 ];

%% 
%Question 2b)


%% 
%Question 2c)


%% 
%Question 2d)
