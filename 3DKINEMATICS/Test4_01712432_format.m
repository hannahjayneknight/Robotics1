%Question 1

%% 
%Question 2


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

clear all; 
close all; 
clc; 

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