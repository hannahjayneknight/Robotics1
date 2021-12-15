function lastframe = rotate3D(frame,angles,R,labels,rotationtype,para)
alpha = angles.alpha;%Z axis rotation angle
beta = angles.beta;%Y axis rotation angle
gamma = angles.gamma;%X axis rotation angle

trans = para.trans;%Transparency of the arrow
width = para.width;%Width of the arrow

x = frame.x;%Unit vectors in the frame
y = frame.y;
z = frame.z;

RR = eval(R);%Obtain the numerical values in the rotation matrix

origin = [0 0 0]';%Location of the origin
vxy_j = [0.5 0.5 0]';%A vector on the x-y plane defined in frame j
vyz_j = [0 0.5 0.5]';%A vector on the y-z plane defined in frame j
vxz_j = [0.5 0 0.5]';%A vector on the x-z plane defined in frame j

vxy_i = RR*vxy_j;%New vector coordinates in the rotated frame
vyz_i = RR*vyz_j;
vxz_i = RR*vxz_j;

%Axis rotation should happen in the opposite direction
alpha = -angles.alpha;
beta = -angles.beta;
gamma = -angles.gamma;

RRaxis = eval(R);%Rotation matrix for the axes of the frame from j to i

xt = RRaxis*x;
yt = RRaxis*y;
zt = RRaxis*z;

%vecs = [x0 xt y0 yt z0 zt]
switch rotationtype
    case 'initialize' %In intialize, I just display the base and the target frames
        plot3([0,0],[0,0],[0,0],'o');hold on;grid on;
        h = mArrow3(origin,z, 'facealpha', trans+0.4, 'color', 'blue', 'stemWidth', width);hold on;
        text(z(1),z(2),z(3)+0.1,'z_0','fontsize',16);% I print the z_0 text with a 0.1 offset from the z coordinate
        h = mArrow3(origin,x, 'facealpha', trans+0.4, 'color', 'red', 'stemWidth', width);
        text(x(1),x(2),x(3)+0.1,'x_0','fontsize',16);
        h = mArrow3(origin,y, 'facealpha', trans+0.4, 'color', 'green', 'stemWidth', width);
        text(y(1),y(2),y(3)+0.1,'y_0','fontsize',16);
        % draws a semitransparent red arrow with a stem width of 0.02 units from point [0 0 0] to point [1 1 1]; h is the handle to the patch object
        
        h = mArrow3(origin,zt, 'facealpha', trans, 'color', 'blue', 'stemWidth', width);hold on;
        text(zt(1),zt(2),zt(3)+0.1,labels{3},'fontsize',16);
        h = mArrow3(origin,xt, 'facealpha', trans, 'color', 'red', 'stemWidth', width);
        text(xt(1),xt(2),xt(3)+0.1,labels{1},'fontsize',16);
        h = mArrow3(origin,yt, 'facealpha', trans, 'color', 'green', 'stemWidth', width);
        text(yt(1),yt(2),yt(3)+0.1,labels{2},'fontsize',16);
        
        axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
        view(135,30);
    case 'individual'
        plot3([0,0],[0,0],[0,0],'o');hold on;grid on;
        h = mArrow3(origin,z, 'facealpha', trans+0.4, 'color', 'blue', 'stemWidth', width);hold on;
        text(z(1),z(2),z(3)+0.1,'z_j','fontsize',16);
        h = mArrow3(origin,x, 'facealpha', trans+0.4, 'color', 'red', 'stemWidth', width);
        text(x(1),x(2),x(3)+0.1,'x_j','fontsize',16);
        h = mArrow3(origin,y, 'facealpha', trans+0.4, 'color', 'green', 'stemWidth', width);
        text(y(1),y(2),y(3)+0.1,'y_j','fontsize',16);
        % draws a semitransparent red arrow with a stem width of 0.02 units from point [0 0 0] to point [1 1 1]; h is the handle to the patch object
        
        h = mArrow3(origin,zt, 'facealpha', trans, 'color', 'blue', 'stemWidth', width);hold on;
        text(zt(1),zt(2),zt(3)+0.1,labels{3},'fontsize',16);
        h = mArrow3(origin,xt, 'facealpha', trans, 'color', 'red', 'stemWidth', width);
        text(xt(1),xt(2),xt(3)+0.1,labels{1},'fontsize',16);
        h = mArrow3(origin,yt, 'facealpha', trans, 'color', 'green', 'stemWidth', width);
        text(yt(1),yt(2),yt(3)+0.1,labels{2},'fontsize',16);
        
        if alpha ~= 0
            h = mArrow3(origin,vxy_j, 'facealpha', trans-0.05, 'color', 'magenta', 'stemWidth', width);hold on;
            text(vxy_j(1),vxy_j(2),vxy_j(3)+0.1,'v_{xy}','fontsize',16);
            title(['\alpha = ',num2str(-alpha*180/pi), '^{\circ}, v_{xy}^{j}: [',num2str(vxy_j'),']^{T}',', v_{xy}^{i}: [',num2str(vxy_i'),']^{T}'],'fontsize',16);
            view(20,70);
        elseif beta ~= 0
            h = mArrow3(origin,vxz_j, 'facealpha', trans-0.05, 'color', 'magenta', 'stemWidth', width);hold on;
            text(vxz_j(1),vxz_j(2),vxz_j(3)+0.1,'v_{xz}','fontsize',16);
            title(['\beta = ',num2str(-beta*180/pi), '^{\circ}, v_{xz}^{j}: [',num2str(vxz_j'),']^{T}',', v_{xz}^{i}: [',num2str(vxz_i'),']^{T}'],'fontsize',16);
        elseif gamma ~= 0
            h = mArrow3(origin,vyz_j, 'facealpha', trans-0.05, 'color', 'magenta', 'stemWidth', width);hold on;
            text(vyz_j(1),vyz_j(2),vyz_j(3)+0.1,'v_{yz}','fontsize',16);
            title(['\gamma = ',num2str(-gamma*180/pi), '^{\circ}, v_{yz}^{j}: [',num2str(vyz_j'),']^{T}',', v_{yz}^{i}: [',num2str(vyz_i'),']^{T}'],'fontsize',16);
        end
        axis([-0.5 1.2 -0.5 1.2 -0.5 1.2]);
    case 'other'%I use this to show steps of axis rotations in a frame
        h = mArrow3(origin,zt, 'facealpha', trans, 'color', 'blue', 'stemWidth', width);hold on;
        text(zt(1),zt(2),zt(3)+0.1,labels{3},'fontsize',16);
        h = mArrow3(origin,xt, 'facealpha', trans, 'color', 'red', 'stemWidth', width);
        text(xt(1),xt(2),xt(3)+0.1,labels{1},'fontsize',16);
        h = mArrow3(origin,yt, 'facealpha', trans, 'color', 'green', 'stemWidth', width);
        text(yt(1),yt(2),yt(3)+0.1,labels{2},'fontsize',16);
        
        axis([-1 1.2 -1 1.2 -1 1.2]);
        view(135,30);
end

lastframe.x = xt;
lastframe.y = yt;
lastframe.z = zt;
