% state_space_derivation.m
%
% Charlie Lambert and Troy Kaufman
% E102 Spring 2024, Prof. Cha, Tsai, Wu, and Yang
%
% Project 2

clear
clc
close all

% create ss from tf
[A B C D] = tf2ss([1], [1 2.5 1])
sys_c = ss(A, B, C, D)

% convert continous to discrete
Ts = 0.1
sys_d = c2d(sys_c, Ts)

% implement controller
Q = [100 0; 0 1];
R = [1];
[K, S, P] = dlqr(sys_d.A ,sys_d.B, Q, R)