%% Minimum Acceleration Trajectory

%% Constants
Tp = 7.8;

% Initial Values
x1 = 0.567;
x1_dot = 0;

% Final Values
x2 = 1.8;
x2_dot = 0;

a1 = x1_dot;
a0 = x1;


%% Creating Required Matrices
T = [Tp^3,   Tp^2;
     3*Tp^2, 2*Tp];

P = [x2 - x1 - x1_dot*Tp;
     x2_dot - x1_dot];
 
 
 %% Calculating the coefficient of the polynomial function
 a = T\P;
 a = [a; a1; a0]
 
 
 %% Plotting the distance, velocity and acceleration profile
 t = 0:0.01:Tp;
 pos = polyval(a, t);
 
 v = polyder(a);
 vel = polyval(v, t);
 acc = polyder(v);
 accel = polyval(acc, t);

hold on;
plot(t, pos); M1 = 'Position Profile';
plot(t, vel); M2 = 'Velocity Profile';
% plot(t, accel); M3 = 'Acceleration Profile';
 
 legend(M1, M2);
 hold off;