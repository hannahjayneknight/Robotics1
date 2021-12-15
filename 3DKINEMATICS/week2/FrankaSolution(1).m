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

%Name 	Joint 1 	Joint 2 	Joint 3 	Joint 4 	Joint 5 	Joint 6 	Joint 7 	Unit
%qmax
%	2.8973 	1.7628 	2.8973 	-0.0698 	2.8973 	3.7525 	2.8973 	rad
%qmin
%	-2.8973 	-1.7628 	-2.8973 	-3.0718 	-2.8973 	-0.0175 	-2.8973 	rad
    
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
%[                  0,                  0,           0,             1]

%Transformation matrix (See lecture notes)
for j=1:TN
    th = qrange(j,1); %Pick each joint angle
    T{j} = [cos(th) -sin(th) 0 DH(j,1);...
        sin(th)*cos(DH(j,3)) cos(th)*cos(DH(j,3)) -sin(DH(j,3)) -DH(j,2)*sin(DH(j,3));...
        sin(th)*sin(DH(j,3)) cos(th)*sin(DH(j,3)) cos(DH(j,3)) DH(j,2)*cos(DH(j,3));...
        0 0 0 1];
end

Tr8 = simplify(T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*T{7}*T{8}); %Compound transformation matrix
Tr7 = simplify(T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*T{7});
Tr6 = simplify(T{1}*T{2}*T{3}*T{4}*T{5}*T{6});
Tr5 = simplify(T{1}*T{2}*T{3}*T{4}*T{5});
Tr4 = simplify(T{1}*T{2}*T{3}*T{4});
Tr3 = simplify(T{1}*T{2}*T{3});
Tr2 = simplify(T{1}*T{2});
Tr1 = simplify(T{1});

Trs{1} = Tr1; Trs{2} = Tr2; Trs{3} = Tr3; Trs{4} = Tr4; Trs{5} = Tr5;
Trs{6} = Tr6; Trs{7} = Tr7; Trs{8} = Tr8;
format short;
tipPos = vpa(Tr8([1:3],4)); %Tip position vector
save tipPos;
save Trs;
disp('Done');
%keyboard;
%%
Jtip = simplify(jacobian(tipPos,qrange(1:7))); %Take derivatives with respect to joint angles to obtain the Jacobian
%%
save Jtip
%%
clc;
clear all;
qmax = [2.8973 	1.7628 	2.8973 	-0.0698 	2.8973 	3.7525 	2.8973]';%Upper limit of join angles
qmin = [-2.8973 	-1.7628 	-2.8973 	-3.0718 	-2.8973 	-0.0175 	-2.8973]';%Lower limit of joint angles

qdotmax = [2.1750 	2.1750 	2.1750 	2.1750 	2.6100 	2.6100 	2.6100]';%Upper limit of joint angular speed
%Question
%Compute trajectory of the tip when all joints rotate at 1/10th of the max
%speed
%Verify it by comparing the results from tipPos and that obtained using the
%Jacobian
load Jtip;
load tipPos;
ang = (qmin+qmax)/2; %Initial conditions at minimum joint range
q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
q6 = ang(6); q7 = ang(7); 
pos = vpa(eval(tipPos))%Initial position
epos = pos; %epos - estimated position
angv = qdotmax/10; %Joint angular speed
ts = 0.01; %Sampling interval
d = [];%Open an empty data matrix
p = [];%Open an empty data matrix
t = [0:ts:1];
for i = 1:length(t)
    ang = ang + ts*angv; %Update joint angle
    
    udiff = qmax - ang; %Check the difference with the max range
    ind = find(udiff < 0);%Find the joints that violate the max range
    ang(ind) = qmax(ind);%Replace them with the max joint angle
    
    ldiff = qmin - ang; %Check the difference with the min range
    ind = find(ldiff > 0); %Find the joints that violate the min range
    ang(ind) = qmin(ind);%Replace them with the min joint angle
    
    q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
    q6 = ang(6); q7 = ang(7); %Update the joint angle variables
    pos = vpa(eval(tipPos)); %Evaluate the tip position
    %vpa(x) uses variable-precision floating-point arithmetic (VPA) to evaluate 
    % each element of the symbolic input x to at least d significant digits, 
    %where d is the value of the digits function. The default value of digits is 32.
    %example vpa(x,d) uses at least d significant digits, instead of the value of digits.
    p = [p;[i*ts pos']]; %Store the tip position vector along with the time
    J = vpa(eval(Jtip)); %Evaluate the Jacobian
    vel = vpa(J*angv); %Obtain the tip velocity
    epos = epos + ts*vel; %Integrate tip velocity to obtain estimated tip position
    d = [d;[i*ts epos']]; %Store the estimated tip position along with time
end
disp('Done')
%%
figure(1);
plot3(d(:,2),d(:,3),d(:,4),'bo');grid on;hold on;
text(d(1,2),d(1,3),d(1,4),'Start','fontsize', 16)
plot3(p(:,2),p(:,3),p(:,4),'r-');
xlabel('x-pos [m]','fontsize', 16);
ylabel('y-pos [m]','fontsize', 16);
zlabel('z-pos [m]','fontsize', 16);
legend('Estimated pos from Jacobian','actual pos','Location','North');
set(gca,'fontsize',16)
%%
% Question
% A) Determine the joint angle trajectories to trace a circle at the tip of
% the robot starting from the position when all the joints are at their mid range. 
% The circle is given by, x = 0.1*sin(2*pi*1.5*t), z = 0.1*cos(2*pi*1.5*t), 
% y  kept constant at initial position, and t is time. Plot for 1 seconds.

clear all;
clc;
ts = 0.01;
qmax = [2.8973 	1.7628 	2.8973 	-0.0698 	2.8973 	3.7525 	2.8973]';%Upper limit of join angles
qmin = [-2.8973 	-1.7628 	-2.8973 	-3.0718 	-2.8973 	-0.0175 	-2.8973]';%Lower limit of joint angles

qdotmax = [2.1750 	2.1750 	2.1750 	2.1750 	2.6100 	2.6100 	2.6100]';%Upper limit of joint angular speed

t = [0:ts:1];
%tpos = [0.1*sin(2*pi*1.5*t);zeros(1,length(t));0.1*cos(2*pi*1.5*t)];
vpos = [0.1*2*pi*1.5*cos(2*pi*1.5*t);...
    zeros(1,length(t));...
    -0.1*2*pi*1.5*sin(2*pi*1.5*t)];%Differentiate the position

load Jtip;
%load tipPos;
ang = (qmin+qmax)/2; %Initial conditions at minimum joint range
q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
q6 = ang(6); q7 = ang(7); 
epos = vpa(eval(tipPos))%Initial position

dth = [];
dJ = [];
for i =1:length(t)
    J = vpa(eval(Jtip)); %Evaluate the Jacobian
    Ji = pinv(J); %Take the pseudo-inverse
    thdot = Ji*vpos(:,i);
    ang = ang + ts*thdot;
    udiff = qmax - ang; %Check the difference with the max range
    ind = find(udiff < 0);%Find the joints that violate the max range
    ang(ind) = qmax(ind);%Replace them with the max joint angle
    
    ldiff = qmin - ang; %Check the difference with the min range
    ind = find(ldiff > 0); %Find the joints that violate the min range
    ang(ind) = qmin(ind);%Replace them with the min joint angle
    
    q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
    q6 = ang(6); q7 = ang(7); %Update the joint angle variables
    dth = [dth;[i*ts ang']];
    dJ = [dJ;[i*ts Ji(:)']]
end

save dth;
save dJ;
%%
% B) Show how the position of each joint moved when the tip traces the
% above circle

clc;
clear all;
load Trs;%This has all intermediate homogeneous transfer functions
load dth; %This contains joint angle profiles when the tip traces the circle
pos = [];
for i = 1:length(dth)
    for j = 1:8
    ang = dth(i,[2:end]);
    q1 = ang(1); q2 = ang(2); q3 = ang(3); q4 = ang(4); q5 = ang(5);
    q6 = ang(6); q7 = ang(7); 
    tip = Trs{j}([1:3],end);
    P = vpa(eval(tip));
    pos(i,j).p = P; 
    end
end
save pos;
%%
load pos;
figure(4);

for i = 1:length(dth)
    joints = [];
    for j = 1:8
        P = pos(i,j).p;
        joints = [joints;P'];
    end
    plot3(joints(:,1),joints(:,2),joints(:,3),'o'); hold on;grid on;
    plot3(joints(:,1),joints(:,2),joints(:,3),'-');
        %text(P(1),P(2),P(3),['Joint-',num2str(i)]);
    drawnow;
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
end

% C) Plot the trajectory of each element in the inverse of the Jacobian
% matrix for the above circle drawing task.

%%

load dth;%This contains joint angle profiles when the tip traces the circle
load dJ;%This contains the trace of all elements of the Jacobian inverse 
%when the tip traces the circle
time = dth(:,1);
figure(2);
plot(time,dth(:,[2:end]),'-','linewidth',3);
xlabel('Time [sec]','fontsize', 16);
ylabel('Joint angles [rad]','fontsize', 16);
legend('Joint-1','Joint-2','Joint-3','Joint-4','Joint-5','Joint-6','Joint-7');

figure(3);
k = 1;
for i = 1:7
    for j = 1:3
        n = 7*(j-1)+i;
        subplot(7,3,k)
        plot(time,dJ(:,n),'-','linewidth',3);
        xlabel('Time [sec]','fontsize', 16);
        ylabel(['Ji(',num2str(i),',',num2str(j),')'],'fontsize', 16);
        k = k + 1;
    end
end

% D) Hence write down any observations about how the joints affect the
% inverse of the Jacobian matrix.
%axis([0.4 0.7 -1.6 -1.4 0.4 0.7]);

