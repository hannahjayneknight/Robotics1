clear all;
clc;
syms m1 m2 th1 th2 thd thd1 thd2 l1 l2 lm1 lm2 I1 I2 g real;
syms k1 k2 real;
syms tau1 tau2 fx fy real;

x1 = lm1*cos(th1);%CoM position of link-1
y1 = lm1*sin(th1);

x2 = l1*cos(th1) + lm2*cos(th1+th2);%CoM position of link-2
y2 = l1*sin(th1) + lm2*sin(th1+th2);

J1 = jacobian([x1;y1],[th1 th2]);
J2 = jacobian([x2;y2],[th1 th2]);

%I1 = (m1*l1^2)/3;
%I2 = (m2*l2^2)/3;
II1 = [I1 0;0 0];%Inertia matrix due to link-1
II2 = [I2 I2;I2 I2];%Inertia matrix due to link-2
H = simplify([II1 + II2 + m1*J1'*J1 + m2*J2'*J2]);%Mass matrix

%V = (dH/dt)thd
%Since there is no direct relationship for dH/dt, I use the chain rule
%dH/dt = (dH/d th1)thd1 + (dH/d th2)thd2
V1 = [jacobian(H(:,1),[th1]) jacobian(H(:,2),[th1])];
V2 = [jacobian(H(:,1),[th2]) jacobian(H(:,2),[th2])];

thd = [thd1;thd2];%The joint velocity vector

T = 0.5*thd'*H*thd;%Kinetic energy

V3 = jacobian(T,[th1; th2]);

V = simplify((V1*thd1 + V2*thd2)*thd - V3');%Coriolis and centrifugal torque

%Gravitational potential energy
U1 = simplify([m1*g*lm1*sin(th1)+m2*g*(l1*sin(th1) + lm2*sin(th1+th2))]);
%Elastic potential energy when the joints have springs
U2 = 0.5*k1*th1^2 + 0.5*k2*th2^2;
U = U1 + U2;%Total potential energy

G = jacobian(U,[th1 th2]);%Gravitational torque

%External force
fext = [fx;fy];
%Torque from external force
tauext = J2'*fext;

%Dynamic equation
state = [thd1;thd2;th1;th2];
A = [zeros(2,4);eye(2) zeros(2)];
tau = [tau1;tau2];
B = [inv(H)*(tau - V - G' - tauext);0;0];
dstate = A*state + B;
%%
clc;
%Joint angle trajectory
T = 0.002;%sampling step size in seconds
traj = [linspace(0, pi/6, 200);linspace(0, pi/2, 200)];
%traj = [linspace(0, pi/6, 200);linspace(0, pi/2, 200)];
dtraj = [zeros(2,1) diff(traj')']/T;
l1 = 0.5;
l2 = 0.6;
lm1 = l1/2;
lm2 = l2/2;
I1 = 0.1;
I2 = 0.2;
m1 = 0.5;
m2 = 0.5;
g = 9.82;
thd1 = 0;
thd2 = 0;
th1 = 0;
th2 = 0;
k1 = 2;
k2 = 2;
kp = 200;
kd = 50;
sval = vpa(eval(state));

d = [];
for i =1:length(traj) %Go through the desired trajectory
    t1 = traj(1,i);
    t2 = traj(2,i);
    dt1 = dtraj(1,i);
    dt2 = dtraj(2,i);
    
    thd1 = sval(1);
    thd2 = sval(2);
    th1 = sval(3);
    th2 = sval(4);
    error = vpa(eval([t1 - th1;t2 - th2]));%Joint angle error
    derror = vpa(eval([dt1 - thd1;dt2 - thd2]));%Joint velocity error
    
    tau1 = kp*error(1) + kd*derror(1);%Computed torque-1
    tau2 = kp*error(2) + kd*derror(2);%Computed torque-2
    
    d = [d;[t1 th1 t2 th2 dt1 thd1 dt2 thd2 tau1 tau2]];
    %Give an angle dependent external force
    %fx = double(10*exp(-(t1-pi/12)^2 - (t2 - pi/4)^2));fy = 0;
    %External force
    fx = double(10*exp(-(t1-pi/8)^2 - (t2 - pi/4)^2));fy = 0;
    
    sval = sval + double(T*vpa(eval(dstate)));
    %[t,s] = ode45(@(t,s) vpa(eval(dstate)),[0 T],state0);
    %sval = s(end,:)';
end

save d;
%%
clear all;
load d;
plot(d);
legend('Angle joint 1','Angle joint 2','Ang. speed joint 1','Ang. speed joint 2');
