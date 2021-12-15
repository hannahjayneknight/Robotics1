clear all;%Clear all variables from memory
syms alpha beta gamma real;%Define symbolid variable to have real numbers

%Rotation by alpha around z axis
Rz = [cos(alpha) -sin(alpha) 0;... %Use ... to go to the next row. It looks better that way
    sin(alpha) cos(alpha) 0;...
    0 0 1];
%Rotation by beta around y axis
Ry = [cos(beta) 0 sin(beta);...
    0 1 0;...
    -sin(beta) 0 cos(beta)];
%Rotation by gamma around x axis
Rx = [1 0 0;...
    0 cos(gamma) -sin(gamma);...
    0 sin(gamma) cos(gamma)];

%First do a X-Y-Z fixed angle rotation to create a target axis frame
Rxyzfixed = simplify(Rz*Ry*Rx);
%%
close all;
clc;
%Unit vectors along each axis
x0 = [1 0 0]';
y0 = [0 1 0]';
z0 = [0 0 1]';
%Then I store them in the frame0 structure
frame0.x = x0;
frame0.y = y0;
frame0.z = z0;
%Angles of rotation around each axis
angles.alpha = pi/6;%z
angles.beta = pi/3;%y
angles.gamma = pi/4;%x

labels = {'x_t','y_t','z_t'};%I use these labels for exes
rotationtype = 'initialize';
para.trans = 0.2;%Transparency of the arrow
para.width = 0.02;%Width of the arrow head
lastframe = rotate3D(frame0,angles,Rxyzfixed,labels,rotationtype,para);%Look inside this function for details of rotation and axis plot
hold on;
%%
clc;
alpha = angles.alpha;
beta = angles.beta;
gamma = angles.gamma;

R = Rxyzfixed;
origin = [0 0 0]';%Location of the origin

epsilon4 = 0.5*sqrt(1 + R(1,1) + R(2,2) + R(3,3));
theta = eval(2*acos(epsilon4));
epsilon1 = (R(3,2)-R(2,3))/(4*epsilon4);
epsilon2 = (R(1,3)-R(3,1))/(4*epsilon4);
epsilon3 = (R(2,1)-R(1,2))/(4*epsilon4);

vec = eval([epsilon1,epsilon2,epsilon3])';
trans = 0.1;
width = 0.02;
h = mArrow3(origin,vec, 'facealpha', trans+0.4, 'color', 'black', 'stemWidth', width);hold on;
