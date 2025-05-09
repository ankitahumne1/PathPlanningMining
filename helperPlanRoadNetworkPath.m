function [referencePath, roadNetwork] = helperPlanRoadNetworkPath(...
    nodes, edges, edgeCosts, edge2pathIdx, cachedPaths,...
    mapMatrix, resolution, referencePathSize, minTurningRadius, startPose, goalPose)
%helperPlanRoadNetworkPath Generates a collision-free smooth path between given
%start and goal poses in a driving environment represented by navGraph &
%occupancy grid map.
%   The road network is represented using the inputs nodes, edges, edgeCosts,
%   edge2pathIdx, cachedPaths. The occupancy grid map is constructed using
%   mapMatrix, resolution. We assume that vehicle takes only forward
%   motion.

% Copyright 2024 The MathWorks, Inc.

arguments
    % Nodes of the graph representing the road network
    % [x, y]
    nodes (:,3) double

    % Edges connecting the nodes of graph representing the road network 
    edges (:,2) double

    % Edge costs
    edgeCosts (:,1) double

    % Index of the cached paths for different edges
    edge2pathIdx (:,1) double

    % Cached paths for each edge in the road network graph
    % [x, y, pathid]
    cachedPaths (:,3) double

    % Occupancy grid map matrix representing the obstacles
    mapMatrix (:,:) double

    % Resolution of the occupancy grid map
    resolution (1,1) double

    % Fixed-size for the reference path output
    referencePathSize (1,1) double

    % Minimum turning radius of the vehicle
    minTurningRadius (1,1) double    

    % Start pose of the vehicle
    startPose (1,3) double

    % Goal pose of the vehicle
    goalPose (1,3) double
end

%--------------------------------------------------------------------------
% Graph A* path planning using the road network
%--------------------------------------------------------------------------
% Create navGraph object for the road network
stateTable = table(nodes, VariableNames="StateVector");
linkTable = table(edges, edgeCosts, edge2pathIdx,...
    VariableNames=["EndStates","Weight","Edge2PathIdx"]);
roadNetwork = navGraph(stateTable, linkTable);

% Create graph-based A* planner object using the navGraph object
routePlanner = plannerAStar(roadNetwork);

% Plan optimal route between start and goal poses on the road network
%   The planner finds the nearest nodes to startPose and goalPose and
%   computes the path between these nodes.
[routeNodes, solnInfo] = plan(routePlanner, startPose, goalPose);
routeNodeIds = solnInfo.PathStateIDs';

% Construct the path along the optimal route using cachedPaths
networkPath = networkPathFromRoute(roadNetwork, cachedPaths, routeNodeIds);

% Smooth the road network path using Savitzky–Golay filter
windowSize = 51; % higher => smoother
polynomialOrder = 2; % lower => smoother
smoothedNetworkPath =  smoothInput(networkPath, windowSize, polynomialOrder);
headingAngle = headingFromXY(smoothedNetworkPath);

%--------------------------------------------------------------------------
% Hybrid A* path planning using occupancy grid map to smoothly enter or
% exit the road network. We assume that vehicle takes only forward motion.
%--------------------------------------------------------------------------
% Create occupancy grid map using occupancy mapMatrix and resolution
map = binaryOccupancyMap(mapMatrix, resolution);

% Create state validator for checking validity of motions
stateSpace = stateSpaceSE2([map.XWorldLimits; map.YWorldLimits; -pi,pi]);
stateValidator = validatorOccupancyMap(stateSpace, Map=map);

% Create Hybrid A* planner
planner = plannerHybridAStar(stateValidator, MinTurningRadius=minTurningRadius);
planner.ReverseCost = realmax; % Avoid reverse paths

% Path planning to enter into the road network path from the start pose
[pathEntry, entryIdx] = planForwardPath(planner, routeNodes, networkPath, headingAngle, startPose, "Entry");

% Path planning to exit from the road network path to the goal pose
[pathExit, exitIdx] = planForwardPath(planner, routeNodes, networkPath, headingAngle, goalPose, "Exit");


%--------------------------------------------------------------------------
% Combine road network paths with paths from Hybrid A* and interpolate
% according to a fixed referencePathSize
%--------------------------------------------------------------------------
% Concatenate road network paths with entry and exit paths from Hybrid A*
% planner
fullpath = [pathEntry(:,1:2);
            smoothedNetworkPath(entryIdx+1:exitIdx-1,:);
            pathExit(:,1:2)];

% Interpolate the path according to a fixed referencePathSize
pathlen = [0; cumsum(nav.algs.distanceEuclidean(fullpath(1:end-1,:), fullpath(2:end,:)))];
referencePath = interp1(pathlen, fullpath, linspace(0,pathlen(end), referencePathSize), 'pchip');
end


function networkPath = networkPathFromRoute(roadNetwork, cachedPaths, routeNodeIds)
% networkPathFromRoute Construct the path along a given route in the
% roadNetwork using cachedPaths

arguments
    % navGraph reprsentation of the road network
    roadNetwork navGraph

    % Cached paths for each edge in the road network graph
    % [x, y, pathid]
    cachedPaths (:,:) double

    % Node IDs corresponding to a route in the roadNetwork
    routeNodeIds(:,1) double
end

% Get edges for the route
routeEdges = [routeNodeIds(1:end-1) routeNodeIds(2:end)];
edgeId = findlink(roadNetwork, routeEdges);

% Get path ids from the edge ids
pathIds = roadNetwork.Links.Edge2PathIdx(edgeId);

% Lookup points in the cachedPath to compute the full path along the route
networkPath = zeros(height(cachedPaths),2);
j = 1;
for i = 1:length(pathIds)
    ind = cachedPaths(:,1)==pathIds(i);
    nsize = sum(ind);
    networkPath(j:j+nsize-1,:) = cachedPaths(ind,2:3);
    j = j+nsize;
end
networkPath(j:end,:) = [];
end


function [pathPoses, nearestIdx] = planForwardPath(planner, routeNodes, networkPath, headingAngle, vehiclePose, entryOrExit, maxAttempts)
% planForwardPath Plan path to enter road network from a start pose or
% exit the road network to a goal pose. We assume that vehicle takes only
% forward motion.
%
% Outputs:
%   - pathPoses: The output path containing the vehicle poses.
%   - nearestIdx: The index of the network path where the output path connects.

arguments
    % plannerHybridAStar planner object
    planner plannerHybridAStar

    % Nodes corresponding to the planned route in the road network
    routeNodes (:,3)

    % Path corresponding to edges connecting the nodes
    networkPath (:,2)

    % Heading angle along the network path
    headingAngle (:,1)

    % The current or goal pose of the vehicle, depending on entry or exit
    vehiclePose (1,3)

    % A flag indicating entry or exit path
    entryOrExit {mustBeMember(entryOrExit, ["Entry", "Exit"])} = "Entry"

    % Max attempts until a path that allows only forward motion is found
    maxAttempts (1,1) double = 5
end

% Find the nearest nodes along the road network path
[~, nearestNodeIds] = sort(nav.algs.distanceEuclidean(vehiclePose(1:2), routeNodes(:,1:2)));

% Attempt until a forward path for entry or exit is found from a nearest
% node on the road network path
for i=1:maxAttempts
    node = routeNodes(nearestNodeIds(i),1:2);
    [~, nearestIdx] = min(nav.algs.distanceEuclidean(node, networkPath));
    nearestPose = [networkPath(nearestIdx,:), headingAngle(nearestIdx)];
    if entryOrExit=="Entry"
        start = vehiclePose;
        goal = nearestPose;
    else
        start = nearestPose;
        goal = vehiclePose;
    end
    % Plan path
    [pathObj, dirs] = plan(planner, start, goal);
    if ~isempty(dirs) && all(dirs==1)
        break
    end
    if i==maxAttempts
        error('%s path not found after %d attempts', entryOrExit, maxAttempts)
    end
end

% Get states data
pathPoses = pathObj.States;
end


function output =  smoothInput(input, windowSize, polynomialOrder)
%smoothInput Smooth input based on with Savitzky–Golay filter.

arguments
    % Input trajectory to be smoothed 
    input (:,:) double

    % Window size for the filter
    windowSize(1,1) {mustBeOdd}

    % Polynomial order < windowSize
    polynomialOrder(1,1) {mustBeInteger}
end

% polynomialOrder: Must be integer & order should be always <= windowSize-1

% Compute the Vandermonde matrix
S = (-(windowSize-1)/2:(windowSize-1)/2)' .^ (0:polynomialOrder);

% Compute QR decomposition
[Q,~] = qr(S,0);

% Compute the projection matrix B
B = Q*Q';

% Compute the transient on
ybegin = B(end:-1:(windowSize-1)/2+2,:) * input(windowSize:-1:1,:);

% Compute the steady state output
ycenter = filter(B((windowSize-1)./2+1,:), 1, input);

% Compute the transient off
yend = B((windowSize-1)/2:-1:1,:) * input(end:-1:end-(windowSize-1),:);

% Concatenate
output = [ybegin; ycenter(windowSize:end,:); yend];

end


function mustBeOdd(input)
%mustBeOdd Validate if the input is odd
validateattributes(input, {'numeric', 'integer'}, {'odd'})
end