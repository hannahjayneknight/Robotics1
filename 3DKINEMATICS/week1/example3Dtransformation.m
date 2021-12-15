clear all;%Clear all variables from memory
%syms alpha pxo pyo pzo real;%Define symbolid variable to have real numbers

alpha = 30 * pi/180;
pxo = 10;
pyo = 5;
pzo = 0;


%Rotation by alpha around z axis
Rz = [cos(alpha) -sin(alpha) 0;... %Use ... to go to the next row. It looks better that way
    sin(alpha) cos(alpha) 0;...
    0 0 1];

po = [pxo;pyo;pzo];

%Homogeneous transformation matrix
T = [Rz po;...
    zeros(1,3) 1]
%%
%Find the real values of the transformation matrix
Tv = vpa(subs(T,{alpha,pxo,pyo,pzo},{pi/6,10,5,0}))
%Then find the new coordinates of Pb = [3;7;0;1] in frame A

V_A = [3; 7 ; 0; 1];
V_B = Tv * V_A