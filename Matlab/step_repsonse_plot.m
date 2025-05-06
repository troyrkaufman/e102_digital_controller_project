% project_01_script.m
%
% Charlie Lambert and Troy Kaufman
% E102 Spring 2024, Prof. Cha, Tsai, Wu, and Yang
%
% Project 2

clear
clc 
close all

% 

% Load the data from csv
data = readtable('step_response.csv');
time  = data.Time*100;
voltage = data.Value;

% Plot the step response
plot(time, voltage)
xlabel('Time (ms)')
ylabel('Voltage (V)')
title('Step Response of Overdamped System')

