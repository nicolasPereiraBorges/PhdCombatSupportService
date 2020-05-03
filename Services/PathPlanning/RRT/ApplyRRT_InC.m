function [points, success] = ApplyRRT_InC(uav, threats, paramOfRRT, virtualTargetFlag)
% function points = ApplyRRT(uav)
% Generate graph using rapidly random tree path planning
% Parameters: uav: Struct uav
% Output: points: path (x, y)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017


% Get all parameters
param = Get_Parameters();
% Get UAV param
uavParam = param.Uav;
% Get simulation parameters
simulationParam = param.Simulation;
% Get path planning parameters
pathPlanningParam = param.PathPlanning;

startPosition = [uav.x, uav.y];
goal = uav.localLeaderPosition;

upperLeft = simulationParam.upperLeft;
lowerRight = simulationParam.lowerRight;

[upperLeft, lowerRight] = CheckScenarioBounds(upperLeft, lowerRight, threats);
sufficientDistanceToGoal = uavParam.sufficientDistanceToGoal;

if  isempty(paramOfRRT) == 1
    thresholdDistanceBase = pathPlanningParam.distanceEdges; 
    thresholdDistance = round(uav.distance*thresholdDistanceBase);    
    if virtualTargetFlag == 0
        numberOfEdges = pathPlanningParam.numberOfEdges;
    else
        numberOfEdges = pathPlanningParam.numberOfEdgesVirtualTargets;
    end
    
else
    thresholdDistanceBase = paramOfRRT.thresholdDistanceBase; 
    numberOfEdges =  paramOfRRT.numberOfEdges; 
    thresholdDistance = round(uav.distance*thresholdDistanceBase);    
end

if thresholdDistance < uav.speed || virtualTargetFlag == 1
    thresholdDistance = uav.speed;
end


c_uav = [startPosition, goal, uav.angle];
if (size(threats,1) > 0)
    threats_cx = [threats.cx]';
    threats_cy = [threats.cy]';
    threats_range = [threats.range]';
    c_threats = [threats_cx, threats_cy, threats_range];
else
    c_threats = [[],[],[]];
end
c_param = [numberOfEdges, thresholdDistance, pathPlanningParam.maxIteractions, pathPlanningParam.angleVariation, upperLeft, lowerRight, sufficientDistanceToGoal];

% Apply RRT in C
points = RRT_C(c_uav, c_threats, c_param);
if size(points,1) == 1
    [x,y] = ProjectPoint(uav.x, uav.y, ...
    uav.localLeaderPosition(1), uav.localLeaderPosition(2), uavParam.speed*simulationParam.timeStep);
    %points = ApplyFixedSpace([x,y; uav.localLeaderPosition], uavParam.speed*simulationParam.timeStep); 
    points = [startPosition;x,y];
    success = 0;
    return;
end
success = 1;

if virtualTargetFlag == 0
    points = ReducePath(points,threats);
end
return;

end


function [upperLeft, lowerRight] = CheckScenarioBounds(upperLeft, lowerRight, threats)

    for i = 1: size(threats, 1)
        threat = threats(i);    
        if threat.cx - threat.range < upperLeft(1)
            upperLeft(1) = threat.cx - (threat.range*1.2);
        end
        if threat.cx + threat.range > lowerRight(1)
            lowerRight(1) = threat.cx + (threat.range*1.2);
        end
        if threat.cy + threat.range > upperLeft(2)
            upperLeft(2) = threat.cy + (threat.range*1.2);
        end
        if threat.cy - threat.range < lowerRight(2)
            lowerRight(2) = threat.cy - (threat.range*1.2);
        end
    end
end