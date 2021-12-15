clear all;
close all;
clc;
origin = [0 0 0]';
%Unit vectors along each axis
x = [1 0 0]';
y = [0 1 0]';
z = [0 0 1]';

trans = 0.1;
width = 0.01;

labels = {' ',' ',' '};

plot3([0,0],[0,0],[0,0],'o');hold on;grid on;
h = mArrow3(origin,z, 'facealpha', trans+0.4, 'color', 'blue', 'stemWidth', width);hold on;
text(z(1),z(2),z(3)+0.1,'z_0','fontsize',16);
h = mArrow3(origin,x, 'facealpha', trans+0.4, 'color', 'red', 'stemWidth', width);
text(x(1),x(2),x(3)+0.1,'x_0','fontsize',16);
h = mArrow3(origin,y, 'facealpha', trans+0.4, 'color', 'green', 'stemWidth', width);
text(y(1),y(2),y(3)+0.1,'y_0','fontsize',16);
axis([-1 1 -1 1 -1 1]);
view(245,35)
hold on;
%%
%z - x linear combinations 
for i = 1:5
    theta = i*2*pi/6;
    cz = sin(theta);
    cx = cos(theta);
    v = cz*z + cx*x;
    
    h = mArrow3(origin,v, 'facealpha', trans, 'color', 'magenta', 'stemWidth', width);hold on;
    text(v(1),v(2),v(3)+0.1,['(',num2str(round(cx,2)),'*x + ',num2str(round(cz,2)),'*z )'],'fontsize',16);
    pause(1);
    drawnow;
end
