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
%Start from the base frame
frame0.x = x0;
frame0.y = y0;
frame0.z = z0;
para.trans = 0.1;
para.width = 0.01;
%Rotation around x0 axis by gamma
rotationtype = 'other';
labels = {' ',' ',' '};
for i = 1:5 %I do it in 5 steps to show intermediate arrows
    angles.alpha = 0;%z
    angles.beta = 0;%y
    %Divide the total angle of pi/4 into 5 steps
    angles.gamma = i*(pi/4)/5;%x 
    if i == 5
        labels = {'x_1','y_1','z_1'};%Label the new axis only at the 5th step for clarity
    end
 
    lastframe1 = rotate3D(frame0,angles,Rxyzfixed,labels,rotationtype,para);
    pause(1);
    drawnow;%Without drawnow, the intermediate steps will be shown at the end of teh loop
end
%%
%Start with the last frame of the previous rotation 
frame0 = lastframe1;
%Rotation around y0 axis by beta
rotationtype = 'other';
labels = {' ',' ',' '};
para.trans = 0.1;
para.width = 0.01;
for i = 1:5
    angles.alpha = 0;%z
    angles.beta = i*(pi/3)/5;%y
    angles.gamma = 0;%x
    if i == 5
        labels = {'x_2','y_2','z_2'};
    end
    lastframe2 = rotate3D(frame0,angles,Rxyzfixed,labels,rotationtype,para);
    pause(1);
    drawnow;
end
%%
%Start with the last frame of the previous rotation 
frame0 = lastframe2;
%Rotation around z0 axis alpha
rotationtype = 'other';
labels = {' ',' ',' '};
para.trans = 0.1;
para.width = 0.01;
for i = 1:5
    angles.alpha = i*(pi/6)/5;%z
    angles.beta = 0;%y
    angles.gamma = 0;%x
    if i == 5
        labels = {'x_t','y_t','z_t'};
    end
    lastframe3 = rotate3D(frame0,angles,Rxyzfixed,labels,rotationtype,para);
    pause(1);
    drawnow;
end



