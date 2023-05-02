close all; clear;

run SetupParams.m
% Control Systems toolbox is needed to run this script

%% Control System Design

% Uncomment the rltool or pidtool for control design
% rltool(G)
% pidtool(G,'pid')

%% Discretize

Ts = 1/5e3;
z = tf('z', Ts);

Gd = c2d(G, Ts);

% Backwards Euler
% Cd_PI = Kp + Ki*Ts*z/(z-1);
% Trapezoidal
Cd_PI = Kp + Ki*(Ts/2)*(z+1)/(z-1);

% Backwards Euler
% Cd_PID = Kp_PID + Ki_PID*Ts*z/(z-1) + (Kd_PID*N*(z-1))/((1+N*Ts)*z-1);
% Trapezoidal
Cd_PID = Kp_PID + Ki_PID*(Ts/2)*(z+1)/(z-1) + (Kd_PID*N*(z-1))/((1+N*(Ts/2))*z + N*(Ts/2)-1);

%% PI step response

SetpointAngularRate = 1;

t = 0:Ts:5;
r = SetpointAngularRate*ones(length(t), 1);

% PI Continuous Step
T_PI = feedback(G*C_PI,1);
y_PI = lsim(T_PI,r,t);
u_PI = lsim(C_PI, r-y_PI, t);

% PI Discrete Step
Td_PI = feedback(Gd*Cd_PI,1);
yd_PI = lsim(Td_PI,r,t);
ud_PI = lsim(Cd_PI, r-yd_PI, t);

% PID Continuous Step
T_PID = feedback(G*C_PID,1);
y_PID = lsim(T_PID,r,t);
u_PID = lsim(C_PID, r-y_PID, t);

% PID Discrete Step
Td_PID = feedback(Gd*Cd_PID,1);
yd_PID = lsim(Td_PID,r,t);
ud_PID = lsim(Cd_PID, r-yd_PID, t); 

figure;
subplot(2,1,1)
plot(t,y_PI, t,yd_PI, t,y_PID, t,yd_PID)
leg = legend('Continuous PI', 'Discrete PI', 'Continuous PID', 'Discrete PID');
set(leg, 'Interpreter', 'latex', 'location', 'southeast');
title('Step Response Plant Output. Ts = 200 us')
ylabel('Plant Output y')
subplot(2,1,2)
plot(t,y_PI-yd_PI, t,y_PID-yd_PID)
leg = legend('PI', 'PID');
set(leg, 'Interpreter', 'latex', 'location', 'southeast');
ylabel('Plant Output Error')
xlabel('time t')
set(gcf,'Position',[100 100 600 600])

figure;
subplot(2,1,1)
plot(t,u_PI, t,ud_PI, t,u_PID, t,ud_PID)
leg = legend('Continuous PI', 'Discrete PI', 'Continuous PID', 'Discrete PID');
set(leg, 'Interpreter', 'latex', 'location', 'northeast');
title('Step Response Control Output. Ts = 200 us')
ylabel('Control Output u')
subplot(2,1,2)
plot(t,u_PI-ud_PI, t,u_PID-ud_PID)
leg = legend('PI', 'PID');
set(leg, 'Interpreter', 'latex', 'location', 'northeast');
ylabel('Control Output Error')
xlabel('time t')
set(gcf,'Position',[100 100 600 600])

%% Generate test cases data

Ts = 1/20; % small Ts
z = tf('z', Ts);
tEnd = 1.5;  % short simulation

Gd = c2d(G, Ts);

% Trapezoidal
Cd_PI = Kp + Ki*(Ts/2)*(z+1)/(z-1);
Cd_PID = Kp_PID + Ki_PID*(Ts/2)*(z+1)/(z-1) + (Kd_PID*N*(z-1))/((1+(N*Ts/2))*z + (N*Ts/2)-1);

SetpointAngularRate = 1;

t = 0:Ts:tEnd;
r = SetpointAngularRate*ones(length(t), 1);

% The yd_PI_test/yd_PID_test and ud_PI_test/ud_PID_test arrays should be coppied and used in the C++ test cases

Td_PI_test = feedback(Gd*Cd_PI,1);
yd_PI_test = lsim(Td_PI_test,r,t);
ud_PI_test = lsim(Cd_PI, r-yd_PI_test, t);

Td_PID_test = feedback(Gd*Cd_PID,1);
yd_PID_test = lsim(Td_PID_test,r,t);
ud_PID_test = lsim(Cd_PID, r-yd_PID_test, t);
