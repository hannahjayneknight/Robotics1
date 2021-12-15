% Alternative-1
clear all;
syms L1 L2 L3 theta1 theta2 real;

%     ai alphai    di thetai
DH = [0     0      0 theta1;...   % Joint 1
      L1 sym(pi)/2 L2 theta2;...   % Joint 2
      L3 0  0 0];     % Joint 3  

 %Now we can separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Finding each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end

T1 = T(1).A;
T2 = T(2).A;
T3 = T(3).A;

T_alt1 = T1*T2*T3
%%
%Alternative-2
clear all;
syms L1 L2 L3 theta1 theta2 real;
%     ai alphai    di thetai
DH = [0     0      0 theta1;...   % Joint 1
      L1 sym(pi)/2 0 theta2;...   % Joint 2
      L3 0  L2 0];     % Joint 3  

 %Now we can separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Finding each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end

T1 = T(1).A;
T2 = T(2).A;
T3 = T(3).A;

T_alt2 = T1*T2*T3
