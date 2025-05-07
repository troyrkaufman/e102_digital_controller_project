% pi_controller_script.m
%
% Charlie Lambert and Troy Kaufman
% E102 Spring 2024, Prof. Cha, Tsai, Wu, and Yang
%
% Project 2

clear
clc 
close all

% Load the data from csv
data = readtable('step_response.csv');
time  = data.Time*100;
voltage = data.Value;

% Define the transfer function
sys = tf([1],[1 2.5 1]);

% Analytical step response
t = (0:0.01:25);
[y,t] = step(sys, t);
t = t*10^3;
y = y*2.5;

% Plot the step response
hold on
plot(time, voltage)
plot(t, y)
xlabel('Time (ms)')
ylabel('Voltage (V)')
title('Analytical and Experimental Step Response')
legend('Experimental', 'Analytical')
