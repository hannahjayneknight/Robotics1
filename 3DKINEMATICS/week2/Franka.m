clear all;
clc;
%https://frankaemika.github.io/docs/control_parameters.html
%+---+-----------+-----------+-----------+-----------+                               
%| j |     a     |         d |     alpha |     theta |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         0 |  0.333    |    0      |      q1   |                               
%|  2|         0 |          0|    -pi/2  |      q2   |                               
%|  3|         0 |  0.316    |    pi/2   |      q3   |                               
%|  4|     0.0825|          0|     pi/2  |      q4   |                               
%|  5|    -0.0825|  0.384    |     -pi/2 |      q5   |                               
%|  6|         0 |          0|      pi/2 |      q6   |                               
%|  7|      0.088|  0        |      pi/2 |      q7   |   
%|Flange|     0  |  0.107    |      0    |      0    |
%+---+-----------+-----------+-----------+-----------+  

%+---+-----------+-----------+-----------+-----------+-----------+
%Name |Joint 1|Joint 2|Joint 3|Joint 4|Joint 5|Joint 6|Joint 7| Unit
%qmax |2.8973 |	1.7628| 2.8973|-0.0698|2.8973 |3.7525 |2.8973 | rad
%qmin |-2.8973|-1.7628|-2.8973|-3.0718|-2.8973|-0.0175|-2.8973| rad
%+---+-----------+-----------+-----------+-----------+-----------+   
syms q1 q2 q3 q4 q5 q6 q7 q8 real; %Symbolic joint angle variables
%pi = sym(pi);

%This DH table is for the last kinematic structure of the slides
% column-1, 2, 3 for 
%|a | d |alpha |
DH = [
    0  0.333  0;...   
    0  0  -pi/2;...   
    0  0.316 pi/2;...        
    0.0825 0 pi/2;...
    -0.0825 0.384 -pi/2;...
    0 1.571 pi/2;...
    0.088 0 pi/2;...
    0 0.107 0];  

TN = length(DH); %Number of frame transformations to be performed

qrange = [q1;q2;q3;q4;q5;q6;q7;0]; %Joint angle vector

%Transformation matrix
%[            cos(th),           -sin(th),           0,             a]
%[ cos(alpha)*sin(th), cos(alpha)*cos(th), -sin(alpha), -d*sin(alpha)]
%[ sin(alpha)*sin(th), sin(alpha)*cos(th),  cos(alpha),  d*cos(alpha)]
% [                  0,                  0,           0,             1]

%Transformation matrix (See lecture notes)
for j=1:TN
    th = qrange(j,1); %Pick each joint angle
    T{j} = [cos(th) -sin(th) 0 DH(j,1);...
        sin(th)*cos(DH(j,3)) cos(th)*cos(DH(j,3)) -sin(DH(j,3)) -DH(j,2)*sin(DH(j,3));...
        sin(th)*sin(DH(j,3)) cos(th)*sin(DH(j,3)) cos(DH(j,3)) DH(j,2)*cos(DH(j,3));...
        0 0 0 1];
end

Tr = simplify(T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*T{7}*T{8}); %Compound transformation matrix

format short;
tipPos = vpa(Tr([1:3],4)) %Tip position vector
%keyboard;
%%
Jtip = simplify(jacobian(tipPos,qrange(1:7))); %Take derivatives with respect to joint angles to obtain the Jacobian
%%
save Jtip
save tipPos
%%
clc;
clear all;
qmax = [2.8973 	1.7628 	2.8973 	-0.0698 	2.8973 	3.7525 	2.8973]';%Upper limit of join angles
qmin = [-2.8973 	-1.7628 	-2.8973 	-3.0718 	-2.8973 	-0.0175 	-2.8973]';%Lower limit of joint angles

qdotmax = [2.1750 	2.1750 	2.1750 	2.1750 	2.6100 	2.6100 	2.6100]';%Upper limit of joint angular speed
%-----------+-----------+-----------+-----------+-----------+-----------+
%Question-1
%Compute trajectory of the tip when all joints rotate at 1/10th of the max
%speed
%Verify it by comparing the results from tipPos and that obtained using the
%Jacobian
%-----------+-----------+-----------+-----------+-----------+-----------+
load Jtip;
load tipPos;
ang = qmin; %Initial conditions at minimum joint range
q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
q6 = ang(6); q7 = ang(7); 
pos = vpa(eval(tipPos))%Initial position
epos = pos; %epos - estimated position
angv = qdotmax/10; %Joint angular speed
T = 0.01; %Sampling interval
d = [];%Open an empty data matrix
p = [];%Open an empty data matrix
for i = 1:100
    ang = ang + T*angv; %Update joint angle
    
    udiff = qmax - ang; %Check the difference with the max range
    ind = find(udiff < 0);%Find the joints that violate the max range
    ang(ind) = qmax(ind);%Replace them with the max joint angle
    
    ldiff = qmin - ang; %Check the difference with the min range
    ind = find(ldiff > 0); %Find the joints that violate the min range
    ang(ind) = qmin(ind);%Replace them with the min joint angle
    
    q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
    q6 = ang(6); q7 = ang(7); %Update the joint angle variables
    pos = vpa(eval(tipPos)); %Evaluate the tip position
    p = [p;[i*T pos']]; %Store the tip position vector along with the time
    J = vpa(eval(Jtip)); %Evaluate the Jacobian
    vel = vpa(J*angv); %Obtain the tip velocity
    epos = epos + T*vel; %Integrate tip velocity to obtain estimated tip position
    d = [d;[i*T epos']]; %Store the estimated tip position along with time
end
disp('Done')
%%
figure;
plot3(d(:,2),d(:,3),d(:,4),'bo');grid on;hold on;
text(d(1,2),d(1,3),d(1,4),'Start','fontsize', 16)
plot3(p(:,2),p(:,3),p(:,4),'r-');
xlabel('x-pos [m]','fontsize', 16);
ylabel('y-pos [m]','fontsize', 16);
zlabel('z-pos [m]','fontsize', 16);
legend('Estiated pos from Jacobian','actual pos','Location','North');
set(gca,'fontsize',16)
%%
%-----------+-----------+-----------+-----------+-----------+-----------+-----------+
% Question-2
% A) Determine the joint angle trajectories to trace a circle at the tip of
% the robot. The circle is give by, z=0, x = 0.2*sin(2*pi*2*t), y =
% 0.2*cos(2*pi*2*t), where t is time. Plot for 3 seconds.
% B) Plot the trajectory of each element in the inverse of the Jacobian
% matrix for the above circle drawing task.
% C) Hence write down any observations about how the joints affect the
% inverse of the Jacobian matrix.
%-----------+-----------+-----------+-----------+-----------+-----------+-----------+
