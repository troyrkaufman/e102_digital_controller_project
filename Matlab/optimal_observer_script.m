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
Q = [100 0; 0 1];
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
