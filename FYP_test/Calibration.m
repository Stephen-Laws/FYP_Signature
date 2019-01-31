clf;
clear all;
res = csvread('calibReadings.csv');
clf;
clc;
clear all;
res = csvread('calibReadings.csv');
res(1,:) = res(1,:);% * pi/2048;
res(2,:) = res(2,:);% * pi/2048;
res(3,:) = res(3,:);% * 25.4/2000;
res(4,:) = res(4,:);% * 12/2000;
for i =1:4
res(i,:) = res(i,:) - res(i,1);
end
t = 0.01:0.01:20;
figure(1);
hold on
plot(t,res(1,:));
plot(t,res(2,:));
hold off
legend('Yaw', 'Pitch','Location','northwest');
xlabel('Time /s');
ylabel('Disturbance /rad');
figure(2)
hold on
plot(t,res(3,:));
plot(t,res(4,:));
hold off
legend('Linear', 'Transverse','location','northeast');
xlabel('Time /s');
ylabel('Disturbance /mm');