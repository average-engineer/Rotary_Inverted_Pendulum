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
%    X = [x0,xA,xP];
%    Y = [y0,yA,yP];
%    Z = [z0,zA,zP];
    
   
   subplot(2,2,3)
   h1 = plot3([x0,xA],[y0,yA],[z0,zA],[xA,xP],[yA,yP],[zA,zP],x0,y0,z0,'o',xA,yA,zA,'o','MarkerFace','r','MarkerFace','b','linewidth',2);
   grid on
   title('Rotary Pendulum System')
   axis([-0.1,0.3,-0.15,0.15,-0.1,0.1])
   
   subplot(2,2,4)
   h2 = plot([x0,xA],[y0,yA],[xA,xP],[yA,yP],x0,y0,'o',xA,yA,'o','MarkerFace','r','MarkerFace','k','Linewidth',2);
   grid on
   title('Rotary Pendulum (XY Plane/Top View)')
   axis([-0.1,0.3,-0.15,0.15])
   
   subplot(2,2,1)
   h3 = plot([x0,xA],[z0,zA],[xA,xP],[zA,zP],x0,z0,'o',xA,zA,'o','MarkerFace','r','MarkerFace','k','Linewidth',2);
   grid on
   title('Rotary Pendulum (ZX Plane/Side View)')
   axis([-0.1,0.3,-0.1,0.1])
   
   subplot(2,2,2)
   h4 = plot([y0,yA],[z0,zA],[yA,yP],[zA,zP],y0,z0,'o',yA,zA,'o','MarkerFace','r','MarkerFace','k','Linewidth',2);
   grid on
   title('Rotary Pendulum (YZ Plane/Front View)')
   axis([-0.15,0.15,-0.1,0.1])
     
   
   pause(0.001)
   delete(h1)  
   delete(h2)
   delete(h3)
   delete(h4)
end


