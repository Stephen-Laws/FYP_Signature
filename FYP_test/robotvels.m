clc;
rotMat = csvread('rotMat.csv');
DHjoints = csvread('circleCoords.csv');

d_1t =0;     t_1 = pi/2;     r_1 = 51.45;     a_1 = -1.509;
d_2t = 0;    t_2t = 1.574;       r_2 = 0;        a_2 = -1.566;
d_3 = 115.5; t_3t = 0;      r_3 = 191.5;      a_3 = 0;

%Place parameters into DH array
DH =    [d_1t, t_1,  r_1, a_1;
         d_2t, t_2t,  r_2, a_2;
         d_3, t_3t, r_3, a_3];
Rpol = [0,0,0,0,0,0];
Rbod = [0,0,0];
vel = zeros(3,length(DHjoints));
for i =1: length(DHjoints)
        DH(1,1) = DHjoints(3,i); %add offset to joint position and place into DH
        DH(2,1) = DHjoints(4,i); 
        DH(2,2) = DHjoints(2,i); 
        DH(3,2) = DHjoints(1,i); 
    T = forwKinematics_d(DH,Rpol,Rbod);
    pos(:,i) = T(1:3);
    if i>112 && i<1100
        tmp =(pos(:,i)-pos(:,i-1));
        vel(1:3,i-112) = tmp*100;
    end
end
plot3(pos(1,:),pos(2,:),pos(3,:))
vel = filter(1,10,vel);
plot(1:1000,vel(:,1:1000))
