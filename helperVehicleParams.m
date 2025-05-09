function vehicleParams = helperVehicleParams
% helperVehicleParams Parameters for vehicle dynamics modeling of large
% mining truck.

% Copyright 2024 The MathWorks, Inc.

% Mass of the vehicle
vehicleParams.mass = 500000; % kg

% Yaw polar inertia
vehicleParams.yawInertia = 1000000; % kg*m^2

% Num wheels front
vehicleParams.numWheelsFront = 2;

% Num wheels rear
vehicleParams.numWheelsRear = 4;

% Distance from CG to front axle
vehicleParams.lengthCGToFrontAxle = 2.5; % meters (approximate)

% Distance from CG to rear axle
vehicleParams.lengthCGToRearAxle = 4.0; % meters

% Height of CG above axles
vehicleParams.heightCG = 2.5; % meters (approximate)

% Minimum turning radius
vehicleParams.minTurningRadius = 15; % meters

% Front tire cornering stiffness
vehicleParams.corneringStiffnessFrontTire = 100000; % N/rad

% Rear tire cornering stiffness
vehicleParams.corneringStiffnessRearTire = 100000; % N/rad

% Maximum steering angle
vehicleParams.maxSteeringAngle = 35 * (pi / 180); % converting 30 degrees to radians

% Maximum steering angle rate
vehicleParams.maxSteeringAngleRate = 45*pi/180;

% Maximum angular velocity (rad/sec)
vehicleParams.maxAngularVelocity = 0.9;

% Max longitudinal acceleration
vehicleParams.maxAcceleration = 0.6;

% Max longitudinal deceleration
vehicleParams.maxDeceleration = -1;

% Max lateral acceleration for comfortable driving
vehicleParams.maxLateralAcceleration = 1.5;

% Vehicle total length
vehicleParams.length = 15;

% Vehicle total width
vehicleParams.width = 9;

% Vehicle model sampling time
vehicleParams.Ts = 1/1000;