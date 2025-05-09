function ipAddress = helperIPAddresses
% helperIPAddresses Define IP addresses for different hardware where the
% Host, Path Follower and Vehicle models will be running

% Copyright 2024 The MathWorks, Inc.

% Rapid Control Prototyping (RCP) system for Path Follower Model
ipAddress.rcpSystem = '172.16.20.122';
% Hardware-in-the-Loop (HIL) System for the Vehicle Model
ipAddress.hilSystem = '172.16.20.83';
% Host computer
ipAddress.hostSystem = '172.16.20.80';