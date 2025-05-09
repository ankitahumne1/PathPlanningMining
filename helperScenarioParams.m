function hostParams =  helperScenarioParams
% helperScenarioParams Define parameters for scenario setup of the host
% model
% Copyright 2024 The MathWorks, Inc.
% Load road network data used for global path planning
% - Nodes, edges and edge weights for graph representing the road network
load("OpenPitMineRoadNetwork.mat", "nodes", "edges", "edgeCosts");
hostParams.nodes = nodes; 
hostParams.edges = edges;
hostParams.edgeCosts = edgeCosts;
% - Cached paths for each edge of road network and lookup from edge to
% cached paths
load("OpenPitMineRoadNetwork.mat", "cachedPaths", "edge2pathIdx");
hostParams.cachedPaths = cachedPaths;
hostParams.edge2pathIdx = edge2pathIdx;
% Load the occupancy map used for global path planning to navigate from an
% initial pose to the road network and exit from the road network to a goal
% pose.
load("OpenPitMineRoadNetwork.mat", "mapMatrix", "resolution");
hostParams.mapMatrix = mapMatrix;
hostParams.resolution = resolution;
% Specify a fixed size for the reference path
hostParams.referencePathSize = 500;
% Host model sampling time
hostParams.Ts = 0.001;
% Initial and goal poses
% hostParams.initPose = [303 440 -pi/4];
% hostParams.goalPose = [390  370 3*pi/4];
hostParams.initPose = [421.5 662.5 -pi/2];
% hostParams.initPose = [503.5 906.5 -3*pi/4];

% hostParams.goalPose = [611.5 675.5 -2*pi/3];

hostParams.goalPose = [677.5 807.5 -2*pi/3];
% hostParams.goalPose = [789.5 362.5 -2*pi/3];
% Get initital translation and rotation for Unreal
load("OpenPitMineRoadNetwork.mat", "mapHeight");
mapHeight = mapLayer(mapHeight, Resolution=resolution);
zOffset = 1.3898e+03;
hostParams.MapSize = [diff(mapHeight.XLocalLimits), diff(mapHeight.YLocalLimits)];
hostParams.Sim3DInitTranslation = [(hostParams.initPose(1:2)-hostParams.MapSize/2),...
                                   mapHeight.getMapData(hostParams.initPose(1:2))-zOffset+14];
hostParams.Sim3DInitRotation = [0,0,hostParams.initPose(3)*180/pi];