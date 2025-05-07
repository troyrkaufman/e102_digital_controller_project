%% Design a unity feedback continuous-time closed loop system for the circuit with a PI compensator
%   C(S) = Kp + Ki/s for 70 degree phase margin and corssover freqeuncy wc0 = 1 rad/s

% Constraints:
% MP < 1%
% zero steady state error
% minimize the control input keeping 1% settling time under 4s
% sample at 10 Hz
% contrl input saturates at 5v

clear all;
clc;

syms Kp Ki real
s = 1j * 1;  % omega = 1 rad/s

% Define the transfer function at s = j1
num = Kp * s + Ki;
den = s * (s^2 + 2.5*s + 1);
Tol = num / den;

% Equations to solve
mag_eq = abs(Tol) == 1;
phase_eq = angle(Tol) * 180/pi == -110;

% Solve both equations
sol = vpasolve([mag_eq, phase_eq], [Kp, Ki]);

% Display results
Kp_val = double(sol.Kp);
Ki_val = double(sol.Ki);

fprintf('Kp = %.4f\nKi = %.4f\n', Kp_val, Ki_val);

% Run simulation
sim('pi_controller_model.slx');

% Extract y and u from the Dataset object
y_signal = yout.getElement('y');   
u_signal = yout.getElement('u');

% Get time and values
y_time = y_signal.Values.Time;
y_data = y_signal.Values.Data;

u_time = u_signal.Values.Time;
u_data = u_signal.Values.Data;

% Plot results
subplot(2,1,1)
plot(y_time, y_data)
title('Digitally Controlled Voltage With PI Compensator')
xlabel('Time [s]')
ylabel('Voltage [V]')

subplot(2,1,2)
plot(u_time, u_data)
title('Plants Input Signal')
xlabel('Time [s]')
ylabel('Voltage [V]')
