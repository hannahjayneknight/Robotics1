clear all;
clc;

syms k1 s1 l1 q1 q2 q3 q4 alpha real; %Symbolic joint angle variables
%pi = sym(pi);

%This DH table is for the last kinematic structure of the slides
% column-1, 2, 3 for 
%|a | d |alpha |
DH = [
    0  0  0;...   %... allows you to go to the next row
    0  0  pi/2;...   
    (2/k1)*sin(s1*k1/2)  0 0;...        
    0 0 -pi/2];  

TN = length(DH); %Number of frame transformations to be performed

%Transformation matrix
%[            cos(th),           -sin(th),           0,             a]
%[ cos(alpha)*sin(th), cos(alpha)*cos(th), -sin(alpha), -d*sin(alpha)]
%[ sin(alpha)*sin(th), sin(alpha)*cos(th),  cos(alpha),  d*cos(alpha)]
% [                  0,                  0,           0,             1]
qrange = [q1; q2; q3; -(pi/2 + alpha/2)];
%Transformation matrix (See lecture notes)
for j=1:TN
    th = qrange(j,1); %Pick each joint angle
    T{j} = [cos(th) -sin(th) 0 DH(j,1);...
        sin(th)*cos(DH(j,3)) cos(th)*cos(DH(j,3)) -sin(DH(j,3)) -DH(j,2)*sin(DH(j,3));...
        sin(th)*sin(DH(j,3)) cos(th)*sin(DH(j,3)) cos(DH(j,3)) DH(j,2)*cos(DH(j,3));...
        0 0 0 1];
end

%q4 = -(pi/2 + alpha/2);
%l1 = (2/k1)*sin(s1*k1/2);

Tr4 = simplify(T{1}*T{2}*T{3}*T{4})

format short;
tipPos = vpa(Tr4([1:3],4)) %Tip position vector
save tipPos;
disp('Done');
%keyboard;
