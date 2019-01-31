function x_0 = enc2pos(d_1, d_2, t_3, t_4)

% enter D-H constants
r_1 = 100;
r_2 = 50;
d_4 = 25;
c_2 = 0.707;
s_2 = 0.707;

% calculate trigonometric values for pitch and yaw
s_3 = sin(t_3);
c_3 = cos(t_3);
s_4 = sin(t_4);
c_4 = cos(t_4);

% enter tooltip position in endframe
x = 20;
y = 70;
z = 0;

x_4 = [x; y; z; 1];

% calculate transformation matrix elements
T = zeros(4)
T(1,1) = s_4;
T(1,2) = -c_4;
T(1,3) = -d_2;
T(2,1) = c_4*(c_2*c_3-s_2*s_3);
T(2,2) = -s_4*(c_2*c_3-s_2*s_3);
T(2,3) = -c_2*s_3-s_2*c_3;
T(2,4) = c_2*r_2+r_1-d_4*(c_2*s_3+s_2*c_3);
T(3,1) = -c_4*(c_2*s_3+s_2*s_3);
T(3,2) = s_4*(c_2*s_3+s_2*c_3);
T(3,3) = s_2*s_3-c_2*c_3;
T(3,4) = d_1-s_2*r_2+d_4*(s_2*s_3-c_2*c_3);
T(4,4) = 1;
% refer to note TH20/34

x_0 = T*x_4;
x_0 = x_0(1:3);
      
end
