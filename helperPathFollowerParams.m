function controllerParams =  helperPathFollowerParams
% helperPathFollowerParams Define parameters for path follower controller

% Copyright 2024 The MathWorks, Inc.

% Lookahead distance for pure pursuit follower
controllerParams.lookaheadDistance = 3; % m

% PID gains for steering control
controllerParams.steeringPGain = 10;
controllerParams.steeringIGain = 0.1;

% PID gains for acceleration control
controllerParams.accelPGain = 10;
controllerParams.accelIGain = 0.1;

% Desired velocity
controllerParams.desiredVelocity = 10; % m/s

% Controller sampling rate
controllerParams.Ts = 1/1000;