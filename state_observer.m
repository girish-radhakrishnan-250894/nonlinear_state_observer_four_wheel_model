function [Q_dot,O_simulator,O_model] = state_observer(t, Q,input)
%vehicle_simulator Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the steering and throttle
%   control action needed and it passes these as scalar values to the
%   vehicle model.

%% Initialization : State Observer variables (only those required to calculate the necessary observer actions)

v_hat = Q(29); % Estimate of lateral velocity - v
r_hat = Q(30); % Estimate of yaq rate - r

%% Initialization : Inputs (reference inputs to be tracked by controller)
delta_c = interp1(input.time, input.delta, t, 'pchip');

m_d_c = 0;

%% Measurement data 
% NOTE - Assuming that the four-wheel model's results can be assumed as a
% sensor data which will be fed to the state-estimator
q = Q(1:28);
[q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q,input,delta_c,m_d_c);


%% Initialization : Measured state

% This is the measured signal that the estimator tries to track
% This could be a signal coming from a sensor but in this code it is coming
% from the four-wheel model that is significantly more complex than the
% estimator model. Therefore, it can be considered to be sensor measurement
r_measured = q_dot(6);


%% Initialization : State Observer 

% Parameters
m = input.m_s;
Izz = input.J_z;
C1 = 103000*2;
C2 = 80000*2;
u = O_model(1);
a = input.a_1;
b = abs(input.a_3);



%% Initializing : Measured output and estimator output

y = r_measured;

y_hat = r_hat;

%% Initializing : Estimator state and estimator input

u = delta_c;

x_hat = [v_hat r_hat]';

%% Estimator Dynamics

x_hat_dot = A*x_hat + B*u + L*(y - y_hat);

%% Augmented system dynamics ([model estimator])

Q_dot = [q_dot;
         x_hat_dot
         ];

%% Initializing outputs to be logged

O_simulator = [O_model(2)];








end