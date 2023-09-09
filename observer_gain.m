%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% OBSERVER GAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -> This script calculates the observer gain matrix L using a linearized
%    version of the bicycle model 

%% State Space Respresentation

% System Matrix
A = -[(C1 + C2)/(m*u),              u + (a*C1 - b*C2)/(m*u);
      (a*C1 - b*C2)/(Izz*u),    (a^2*C1 + b^2*C2)/(Izz*u)];

% Input Matrix
B = [C1/m;
     a*C1/Izz];

% Output Matrix
C = [0 1];

%% State Observer Gain
% Using pole placement technique to create the observer gain matrix
% Using the principle of duality, (A-BK) is equivalent to (A' - C'L').
% Therefore, A', C' can be used using the placeMIMO function (which is the
% same as the place.m function of
L = placeMIMO(A',C',[-15,-17])';