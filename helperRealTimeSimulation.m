function target = helperRealTimeSimulation(model)
%helperRealTimeSimulation Connect to real-time target and start the
%real-time simulation

% Copyright 2024 The MathWorks, Inc.
ipAddress = helperIPAddresses;

% Create Simulink Real-Time object
target = slrealtime;

if model =="PathFollower"
    % Set IP address of the target
    setipaddr(target,ipAddress.rcpSystem,'255.255.0.0')
end
if model =="Vehicle"
    % Set IP address of the target
    setipaddr(target,ipAddress.hilSystem,'255.255.0.0')
end

% Connect to target
connect(target);

% Read the system target file for the target
modelSTF = getSTFName(target);

% Set system  parameter in the target model
load_system(model)
set_param(model,"SystemTargetFile", modelSTF);
save_system(model);

% Build the target model
slbuild(model);

% Load the controller model to the target
load(target,model);

% Start the loaded application on the target machine.
start(target);