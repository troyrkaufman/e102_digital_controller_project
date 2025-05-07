%% Design a unity feedback continuous-time closed loop system for the circuit with a PI compensator
%   C(S) = Kp + Ki/s for 70 degree phase margin and corssover freqeuncy wc0 = 1 rad/s

% important functions margin(Tol)

% Constraints:
% MP < 1%
% zero steady state error
% minimize the control input keeping 1% settling time under 4s
% sample at 10 Hz
% contrl input saturates at 5v

syms Kp Ki real
s = 1j * 1;  % omega = 1 rad/s

% Define the transfer function at s = j1
numerator = Kp * s + Ki;
denominator = s * (s^2 + 2.5*s + 1);
Tjw = numerator / denominator;

% Equations to solve
mag_eq = abs(Tjw) == 1;
phase_eq = angle(Tjw) * 180/pi == -110;

% Solve both equations
sol = vpasolve([mag_eq, phase_eq], [Kp, Ki]);

% Display results
Kp_val = double(sol.Kp);
Ki_val = double(sol.Ki);

fprintf('Kp = %.4f\nKi = %.4f\n', Kp_val, Ki_val);
