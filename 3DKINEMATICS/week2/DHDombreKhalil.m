clear all;
syms th d a alpha real;%These are the DH parameters for joint-i

%Rotation matrix around z_i axis
rotation_z = [cos(th) -sin(th) 0;...
              sin(th) cos(th) 0;...
              0 0 1]
%Translation along z_i axis
translation_z = [0 0 d]'

%The screw on z_i axis. A Screw is a transformation matrix where rotation
%and translation happen on just one axis.
screw_z = [ rotation_z translation_z;...
            0 0 0 1]
        

%Rotation matrix around x_(i-1) axis     
rotation_x = [1 0 0;...
              0 cos(alpha) -sin(alpha);...
              0 sin(alpha) cos(alpha)]
%Translation along x_(i-1) axis         
translation_x = [a 0 0]'

%The screw on x_(i-1) axis
screw_x = [ rotation_x translation_x;...
            0 0 0 1]  
        
%The homogeneous transformation matrix
T = screw_x*screw_z
        
