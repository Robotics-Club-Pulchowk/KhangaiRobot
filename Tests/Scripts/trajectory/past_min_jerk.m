%% Minimum Jerk Trajectory

%% Constants
Tp = 7.8;

% Initial Values
p1 = [0.575];
p1_dot = [0];

% Final Values
p2 = [1.8];
p2_dot = [0];

a2 = [0];
a1 = p1_dot;
a0 = p1;


%% Creating Required Matrices
T = [Tp^5, Tp^4, Tp^3;
     5*Tp^4, 4*Tp^3, 3*Tp^2;
     20*Tp^3, 12*Tp^2, 6*Tp];

P = [p2 - p1 - p1_dot*Tp;
     p2_dot - p1_dot;
     zeros(1, 1)];
 
 
 %% Calculating the coefficient of the polynomial function
 a = T\P;
 a = [a; a2; a1; a0]
 
 
 %% Plotting the distance, velocity and acceleration profile
 t = 0:0.01:Tp;
 pos = polyval(a, t);
%  posY = polyval(a(:,2), t);
 
 v = polyder(a);
 vel = polyval(v, t);
 acc = polyder(v);
 accel = polyval(acc, t);
%  j = polyder(acc);
%  jerk = polyval(j, t);
%
hold on;
plot(t, pos); M1 = 'Position Profile';
plot(t, vel); M2 = 'Velocity Profile';
% plot(t, accel); M3 = 'Acceleration Profile';
% axis equal;
%plot(t, jerk); M4 = 'Jerk Profile';
 
%  legend(M1, M2, M3);
 hold off;