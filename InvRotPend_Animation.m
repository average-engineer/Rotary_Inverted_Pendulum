function InvRotPend_Animation(w,L,l)
% Animating the response of an inverted rotary pendulum system

% Motor Pivot Coordinates
x0 = 0;
y0 = 0;
z0 = 0;

%% 2D Animation
% figure
% for i = 1:max(size(w))
%     subplot(1,2,1)
%     xA = L*cos(w(i,1));
%     yA = L*sin(w(i,1));
%     
%     h1 = plot([x0,xA],[y0,yA],x0,y0,'o','MarkerFace','r','linewidth',2);
%     axis([-0.1,0.1,-0.02,0.02])
%     title('Rotary Arm(Global XY Plane)')
%     grid on
%     
%     subplot(1,2,2)
%     zB = l*cos(w(i,2));
%     yB = -l*sin(w(i,2));
%     
%     h2 = plot([z0,zB],[y0,yB],z0,y0,'o','MarkerFace','r','linewidth',2);
%     axis([-0.2,0.2,-0.05,0.05])
%     title('Inverted Pendullum(Plane normal to arm longitude)')
%     grid on
%     pause(0.1)
%     delete(h1)
%     delete(h2)
% end

%% 3D Animation
% All coordinates are wrt global intertial frame

figure
for i = 1:max(size(w))
   % Rotary Arm and Pendulum connection coordinates
   xA = x0 + L*cos(w(i,1));
   yA = y0 + L*sin(w(i,1));
   zA = z0;
   
   % Inverted Pendulum free end coordinates
   xP = xA + (l/2)*sin(w(i,1))*sin(w(i,2));
   yP = yA - (l/2)*sin(w(i,2))*cos(w(i,1));
   zP = zA + (l/2)*cos(w(i,2));
   
   % 3D coordinates
   X = [x0,xA,xP];
   Y = [y0,yA,yP];
   Z = [z0,zA,zP];
   
   h1 = plot3(X,Y,Z,x0,y0,z0,'o',xA,yA,zA,'o');
   grid on
   title('Rotary Pendulum System')
   axis([-0.1,0.3,-0.3,0.3,-0.2,0.2])
   pause(0.1)
   delete(h1)  
end


