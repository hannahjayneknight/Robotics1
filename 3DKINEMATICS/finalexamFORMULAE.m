% FINAL EXAM FORMULAE

%% 
%Transformation matrix 
% replace i with numbers according to which transformation, then multiply 
% all together to get the homogenous matrix

syms thetai alphai di ai;
jTi = [ cos(thetai), -sin(thetai), 0, ai;
        sin(thetai)*cos(alphai), cos(thetai)*cos(alphai), -sin(alphai), -sin(alphai)*di;
        sin(thetai)*sin(alphai), cos(thetai)*sin(alphai), cos(alphai), cos(alphai)*di;
        0, 0, 0, 1 ];

%% 
%Jacobian stuff