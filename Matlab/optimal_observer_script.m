% optimal_observer_script.m
%
% Charlie Lambert and Troy Kaufman
% E102 Spring 2024, Prof. Cha, Tsai, Wu, and Yang
%
% Project 2

clear
clc
close all

% create ss from tf
% [A B C D] = tf2ss([1], [1 2.5 1])
r = 50*10^3;
c = 10*10^-6
A = [-1/(4*r*c) 1/(4*r*c); 1/(4*r*c) (-1/(r*c)-1/(4*r*c))]
B = [0; 1/(r*c)]
C = [1 0]
D = [0]
sys_c = ss(A, B, C, D)
step(sys_c)

% convert continous to discrete
Ts = 0.1
sys_d = c2d(sys_c, Ts)

% solve for K and poles
Q = [20000 0; 0 50];
R = [1];
[K, S, P] = dlqr(sys_d.A ,sys_d.B, Q, R)

% solve reference gain
Kr = -inv((sys_c.C-sys_c.D*K)*inv(sys_d.A-eye(2)-sys_d.B*K)*sys_d.B-sys_c.D)

% solve observer gains
P_o = 0.5*P
L = place(sys_d.A', sys_d.C', P_o)'

% Load and run the simulink model
load_system('optimal_observer_model.slx')
simOut = sim('optimal_observer_model.slx', 'SaveOutput', 'on', 'OutputSaveName', 'yout');

% Extract signals
logs = simOut.yout;
% theta = logs.get('theta');
% theta_time = theta.Values.Time;
% theta_data = theta.Values.Data;
% s = logs.get('s');
% s_time = s.Values.Time;
% s_data = s.Values.Data;
% a = logs.get('a');
% a_time = a.Values.Time;
% a_data = a.Values.Data;
% step = logs.get('step');
% step_time = step.Values.Time;
% step_data = step.Values.Data;

arduino_data = readtable('optimal_observer_control.csv');
time_ard = arduino_data.Time*0.1;
y_ard = arduino_data.Output;
u_ard = arduino_data.Input;

sim_y = logs.get('y')
sim_u = logs.get('u')
y_time = sim_y.Values.Time;
y_data = sim_y.Values.Data;

u_time = sim_u.Values.Time;
u_data = sim_u.Values.Data;

figure(1)
% Plot results
subplot(2,1,1) 
hold on
plot(y_time, y_data)
% plot(time_ard, y_ard)
% legend('Simulated', 'Experimental')
title('Output With Optimal Control and Observer')
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([0 5])

hold off
subplot(2,1,2)
hold on
plot(u_time, u_data)
% plot(time_ard, u_ard)
% legend('Simulated', 'Experimental')
title('Control Input To The Plant')
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([0 5])
hold off 

fprintf('{%.4f, %.4f, %.4f, %.4f, %.4f}\n', Kr, K(1), K(2), L(1), L(2));

