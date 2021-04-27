function scenario = FlyNew(scenario, stealth)
global VISITED_GF;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global REPORT_EXPOSURE_TIME;
%global SAFETY_FLY_ZONE;
global RISK_MAP;

global BEST_SOLUTIONS;
global BEST_FITNESS;

param = Parameters();
nUAVs = scenario.UAVs.Count();

MINDIST = 10;
% For each UAV
for i = 1: nUAVs
    % Capture UAV
    uav = scenario.UAVs.Value(i);   
    % Check if uav is destroyed or in GCS
    if uav.IsUnderMission() == 0
        continue;
    end
    % Check if UAV arrived GCS
    if uav.GroundForcesToVisit.Count() == 0 && IsUAVCloseToPosition(uav, scenario.GCS.Position) == 1        
        uav.UnderMission = 0;
        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);
        continue;
    end    
    lastExp = uav.Exposed;
    % Detect Threats
    [scenario, ~, ~] = DetectThreats(scenario, i, stealth);    
    uav = scenario.UAVs.Value(i);
    % Check if uav is exposed to a theat
    if (uav.Exposed == 1)
        % UAV exposed to threat
        threat = uav.Threats.Last();        
        % Update Risk map
        UpdateRiskMapWithThreat(threat, uav.Id);
        
        if (uav.Alive == 0 || EXPOSURE_TIME(i) == param.ExposureTimeThreshold)
                %uav.Alive = 0;                
                scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);                
                lastPath = uav.FlightPath;
                tt = scenario.Threats;                               
                scenario = DVRPOptimizer(scenario);
                
                uav = scenario.UAVs.Value(i);
                uav.FlightPath = lastPath;
               % uav.Alive = lastAlive;
                scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);
                scenario.Threats = tt;
                return;
        end                        
    else
        % UAV outside threat region
        RISK_MAP = UpdateSafeZone(RISK_MAP, uav.Position.X, uav.Position.Y);
        AVAILABLE_UAVS(i) = 1;
        EXPOSURE_TIME(i) = 0;
        if (lastExp == 1 &&  stealth  == 1 && uav.GroundForcesToVisit.Count()==0)
            % UAV just leave threat region
            %scenario = DVRPOptimizer(scenario);            
            return;
            %continue;
        end
    end
            
    % Update Path planning    
    if uav.FlightPath.Count() > 0      
                goal = uav.GetNextGoalPosition(scenario);                
                [newGoal] = GetNextGoalUAVRiskMap(uav,goal);
                newGoal = newGoal.AddLast(goal);
                uav.WaypointsToVisit = newGoal;
                % Check distante between UAV and goal
                if IsUAVCloseToPosition(uav, goal) == 1
                    % UAV is close enough to goal
                    idGroundForce = uav.GroundForcesToVisit.First();
                    uav.FlightPath = FlightPath();
                    if ~isempty(idGroundForce)
                        % GF is visited                        
                        gf = scenario.GroundForces.Value(idGroundForce);
                        gf.Visited = 1;                        
                        % Get next GF
                        scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);
                        uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);                        
                        VISITED_GF(gf.Id) = 1;
                        scenario = DVRPOptimizer(scenario);  
                        continue;                                                                       
                    end         
                else
                    % Update UAV position
                    if (GoalInsideAThreat(uav, uav.FlightPath.First()) == 0 || uav.Exposed == 1)
                        uav.Position = uav.FlightPath.First();
                    else
                        uav = ApplyRRT_InC(uav, uav.FlightPath.Last());
                        uav.Position = uav.FlightPath.First();            
                    end
                    % Remove first position of Fligth Path
                    uav.FlightPath.Positions = uav.FlightPath.Positions.RemoveValueInIndex(1);
                    uav = uav.UpdatePathHistory();
                end
    else
        % Fligth path is empty
        if param.RiskMap == 1
            % If there is no waypoint
            if (uav.WaypointsToVisit.Count() == 0)                
                % Check next Goal position
                goal = uav.GetNextGoalPosition(scenario);                
                [newGoal] = GetNextGoalUAVRiskMap(uav,goal);
                newGoal = newGoal.AddLast(goal);
                uav.WaypointsToVisit = newGoal;
                % Check distante between UAV and goal
                if IsUAVCloseToPosition(uav, goal) == 1
                    % UAV is close enough to goal
                    idGroundForce = uav.GroundForcesToVisit.First();
                    uav.FlightPath = FlightPath();
                    if ~isempty(idGroundForce)
                        % GF is visited                        
                        gf = scenario.GroundForces.Value(idGroundForce);
                        gf.Visited = 1;
                        VISITED_GF(gf.Id) = 1;
                        
                        % Get next GF
                        scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);
                        uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);  
                        %scenario = DVRPOptimizer(scenario);  
                        continue;                                                                       
                    end                    
                else
                    % UAV not arrived in destination                      
                    if (GoalInsideAThreat(uav, newGoal.First()) == 0)
                        uav = ApplyRRT_InC(uav, newGoal.First());
                    else
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);                        
                    end
                    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);                        
                    continue;     
                end
            else
                % There are whaypoints to visit
                % Get next Waypoint
                goal = uav.WaypointsToVisit.First();
                dist = uav.Position.CalculateDistancePos(goal);
                if dist < MINDIST
                    uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                    uav.FlightPath = FlightPath();
                else
                    if (GoalInsideAThreat(uav, goal) == 0)
                        uav = ApplyRRT_InC(uav, goal);
                    else
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                       
                    end
                    
                end
            end
        else
            goal = uav.GetNextGoalPosition(scenario);
            dist = uav.Position.CalculateDistancePos(goal);
            if dist < MINDIST
                idGroundForce = uav.GroundForcesToVisit.First();
                if ~isempty(idGroundForce)
                    gf = scenario.GroundForces.Value(idGroundForce);
                    gf.Visited = 1;
                    VISITED_GF(gf.Id) = 1;
                    scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);
                    uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                    
                end
                uav.FlightPath = FlightPath();
            else
                if (GoalInsideAThreat(uav, goal) == 0)
                    uav = ApplyRRT_InC(uav, goal);
                end
            end
            
        end
    end
    
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);

end
end

function UpdateRiskMapWithThreat(threat, uavId)
global RISK_MAP;
global REPORT_EXPOSURE_TIME;
global AVAILABLE_UAVS;
global EXPOSURE_TIME;

 RISK_MAP = UpdateDetectionProbability(RISK_MAP, threat);
 REPORT_EXPOSURE_TIME = REPORT_EXPOSURE_TIME + 1;
 AVAILABLE_UAVS(uavId) = 0;
 EXPOSURE_TIME(uavId) = EXPOSURE_TIME(uavId) + 1;

end

function stop = StopExecution(scenario)
global VISITED_GF;

nUAVs = scenario.UAVs.Count();
for i = 1: nUAVs
    uav = scenario.UAVs.Value(i);
    
    if uav.GroundForcesToVisit.Count() > 0 || sum(VISITED_GF==0) > 0
        %if uav.UnderMission == 1
        stop = 0;
        return;
    end
end

stop = 1;
end



function [inside] = GoalInsideAThreat(uav, goal)

threats = uav.Threats;
for i = 1: threats.Count()
    p = threats.Value(i).Position;
    dist = goal.CalculateDistancePos(p);
    if (dist <= threats.Value(i).DetectionRange)
        inside = 1;
        return;
    end
end
inside = 0;

end

function res = IsUAVCloseToPosition(uav, pos)
    
    res = pos.CalculateDistancePos(uav.Position) < 100;
end


function UpdateCostMapThreats(scenario)
global NO_FLY_ZONE;

threats = scenario.Threats;
nThreats = threats.Count();

n = size(NO_FLY_ZONE);

c = (1:n(2))';

for i = 1: n(1)
    d = zeros(n(2),1) + i;
    pos = [d,c];
    for j = 1: nThreats
        threat = threats.Value(j);
        if (threat.Detected == 1)
            p = threat.Position;
            dists = sqrt( (pos(:,1) - p.X).^2 + (pos(:,2) - p.Y).^2);
            [ll,cc] = find(dists.*1.1 < threat.DetectionRange);
            if (~isempty(ll))
                pp = pos(ll,:) + 1000;
                %COST_MAP(pp(:,1), pp(:,2)) = 100;
                NO_FLY_ZONE(pp(:,1), pp(:,2)) = 100;
            end
        end
    end
    
end

end

function GenerateLog(message)
return;
global LOG;
global TIME;

LOG = "Time: " + num2str(TIME) +  " sec -> " + message + "\n"+LOG;
if length(LOG) > 500
    LOG = LOG(1:500);
end

end