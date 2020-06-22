function uav = ApplyRRT_InC(uav, goal)
% function points = ApplyRRT(uav)
% Generate graph using rapidly random tree path planning
% Parameters: uav: Struct uav
% Output: points: path (x, y)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017


% Get all parameters
param = Parameters();
angleRef = param.angleVariation;

factor = 200;
startPosition = uav.Position.GetAsArray2D();
upperLeft = [-factor, param.ScenarioHeight + factor];
lowerRight = [param.ScenarioWidth+factor, -factor];
sufficientDistanceToGoal = 50;
maxIteractions = 50000;
thresholdDistance = uav.Position.CalculateDistancePos(goal)*0.3;
%c_uav = [startPosition, goal, uav.Angle];
c_uav = [startPosition, goal.GetAsArray2D(), angleRef];
threats = uav.Threats;

nThreats = threats.Count();
if (threats.Count() == 0)
    c_threats = [[],[],[]];
else
    c_threats = zeros(nThreats,3);
    for i = 1:nThreats
       c_threats(i,1:3) = threats.Value(i).GetAsArray(); 
       c_threats(i,3) = c_threats(i,3) * 1.1;
    end
end

numberOfEdges = 10000;
%c_param = [numberOfEdges, thresholdDistance, maxIteractions, uav.Angle, upperLeft, lowerRight, sufficientDistanceToGoal];
c_param = [numberOfEdges, thresholdDistance, maxIteractions, angleRef, upperLeft, lowerRight, sufficientDistanceToGoal];

% Apply RRT in C
%points = RRT_C(c_uav, c_threats, c_param);
%points = ReducePath(points,threats);

if ~LineIntersectsObstacle(startPosition, goal.GetAsArray2D(), c_threats) && ...
    AngleIsInRange(startPosition, goal.GetAsArray2D(), angleRef)
    points = [startPosition; goal.GetAsArray2D()];
else    
    points = RRT_C(c_uav, c_threats, c_param);
    points = ReducePath(points,c_threats);
    points = ApplyPathSmoothing(points);
end
points = ApplyFixedSpace(points, uav.Speed); 

uav.FlightPath = uav.FlightPath.UpdatePosGivenArray(points);

 
return;

% if size(points,1) == 1
%     [x,y] = ProjectPoint(uav.x, uav.y, ...
%     uav.localLeaderPosition(1), uav.localLeaderPosition(2), uavParam.speed*simulationParam.timeStep);
%     %points = ApplyFixedSpace([x,y; uav.localLeaderPosition], uavParam.speed*simulationParam.timeStep); 
%     points = [startPosition;x,y];
%     success = 0;
%     return;
% end
% success = 1;
% 

% return;

end

