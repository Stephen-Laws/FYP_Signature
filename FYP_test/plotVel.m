clc;
cla;clf
figure(1)
h = animatedline;
view([-184.9 -61.2])
axis([-100 100 80 140 300 350]);

daspect([1 1 1]);


pos = csvread('lineSimPos.csv');
vel = csvread('test.csv');
pos2 = csvread('flinePos.csv');
vel2 = csvread('flineVel.csv');
pos3 = csvread('linePosAct.csv');
vel3 = csvread('lineVelSim.csv');

%hh1(1) = line(T(1), ang(1,1), 'Marker', '.', 'MarkerSize', 20, 'Color', 'b');
x = pos3(1,:);
y = pos3(2,:);
z = pos3(3,:);


F(length(x)) = struct('cdata',[],'colormap',[]);
input('y')
for i =1:length(x)
    addpoints(h,x(i),y(i),z(i))
    drawnow;
end


% figure(1)
% subplot(2,1,1)
% plot3(pos(1,:),pos(2,:),pos(3,:))
% daspect([1 1 1])
% title('Simulated trajectory');
% xlabel('x')
% ylabel('y')
% zlabel('z')
% subplot(2,1,2)
% plot3(pos3(1,:),pos3(2,:),pos3(3,:))
% daspect([1 1 1])
% title('Actual trajectory');
% xlabel('x')
% ylabel('y')
% zlabel('z')

%subplot(2,1,2)
%plot3(pos3(1,:),pos3(2,:),pos3(3,:))
% view([180, -90])
% daspect([1 1 1])
% title('Actual Position')
% xlabel('x')
% ylabel('y')
% zlabel('z')
t = 0:0.005:length(pos2)/200-0.005;
% 
% figure(2)
% hold on
% stairs(t,vel2(1,:),'r')
% stairs(t,vel2(2,:),'b')
% stairs(t,vel2(3,:),'g')
% stairs(t,vel2(4,:),'k')
% 
% for i =1:length(pos)
%     for j = 1:4
%         tmp(j) = mean(vel2(j,20*(i-1)+1:20*(i-1)+20))';
%     end
% error(:,i) = vel3(:,i) - tmp';
% end
% t = 0:0.1:length(vel3)/10-0.1;
% %subplot(4,1,1)
% % 
% % plot(t,error(1,:),'r')%,'linewidth',1)
% % title('Linear')
% % ylabel('Velocity /rpm')
% % xlabel('Time /s')
% % subplot(4,1,2)
% % 
% % 
% % plot(t,error(2,:),'b')'linewidth',1)
% % title('Transverse')
% % ylabel('Velocity /rpm')
% % xlabel('Time /s')
% % subplot(4,1,3)
% % 
% % plot(t,error(3,:),'g')%,'linewidth',1)
% % ylabel('Velocity /rpm')
% % title('Pitch')
% % xlabel('Time /s')
% % subplot(4,1,4)
% % 
% % plot(t,error(4,:),'k')%,'linewidth',1)
% % ylabel('Velocity /rpm')
% % title('Yaw')
% % xlabel('Time /s')
% 
% stairs(t,vel3(1,:),'r','linewidth',1.5)
% stairs(t,vel3(2,:),'b','linewidth',1.5)
% stairs(t,vel3(3,:),'g','linewidth',1.5)
% stairs(t,vel3(4,:),'k','linewidth',1.5)
% legend('d_1','d_2','\theta_2','\theta_3')
% ylabel('Velocity /rpm')
% xlabel('Time /s')

%encReadings = csvread('currArray.csv');
