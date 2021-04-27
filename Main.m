% Perform complete simulation of Combat Support Service logistical support

% Global parameters
global LOG;
global TIME;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global VISITED_GF;
global RISK_MAP;

% Start parameters
LOG = "";
TIME = 0;
VISITED_GF = [];
EXPOSURE_TIME = [];
AVAILABLE_UAVS = [];
nUAVs = 2;
cellSizeRiskMap = 200;

% Load scenario
%load('Z:\Doutorado\Source\Dataset_22_7_2020\T=8\G=10\8.mat');
load('Z:\Doutorado\Source\Dataset_22_7_2020\T=8\G=5\8.mat');
% Add UAVs to scneario
scenario.UAVs =  CreateUavs(scenario.GCS, nUAVs);
% Create risk map
param = Parameters();
RISK_MAP = CreateRiskMap(param.ScenarioHeight, param.ScenarioWidth, cellSizeRiskMap);
% Perform simulation
RunSimulation(scenario)

% close(video2);

