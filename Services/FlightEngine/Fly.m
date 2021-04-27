function scenario = Fly(scenario, stealth)
global VISITED_GF;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global REPORT_EXPOSURE_TIME;
%global SAFETY_FLY_ZONE;
global RISK_MAP;

param = Parameters();

nUAVs = scenario.UAVs.Count();

% Para cada UAV
for i = 1: nUAVs
    % Captura UAV
    uav = scenario.UAVs.Value(i);
    
    if (uav.Alive == 0 || uav.UnderMission == 0)
        continue;
    end
    
    if uav.GroundForcesToVisit.Count() == 0 && scenario.GCS.Position.CalculateDistancePos(uav.Position) < 50
        uav.UnderMission = 0;
        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);
    end
    
    lastExp = uav.Exposed;
    % Detect Threats
    [scenario, ~, ~] = DetectThreats(scenario, i, stealth);    
    uav = scenario.UAVs.Value(i);
    if (uav.Exposed == 1)
        % UAV exposed to threat
        threat = uav.Threats.Last();        
        RISK_MAP = UpdateDetectionProbability(RISK_MAP, threat);
        REPORT_EXPOSURE_TIME = REPORT_EXPOSURE_TIME + 1;
        AVAILABLE_UAVS(i) = 0;
        EXPOSURE_TIME(i) = EXPOSURE_TIME(i) + 1;
        if stealth  == 1
            if (EXPOSURE_TIME(i) == param.ExposureTimeThreshold) ...
                    || (uav.Alive==0)
                scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);
                tt = scenario.Threats;                
                %lastThreat = scenario.Threats.Last();                
                %scenario = Replan_Remove_DVRP(scenario, i);
                scenario = DVRPOptimizer(scenario);
                scenario.Threats = tt;
                return;
                %uav = scenario.UAVs.Value(i);
            end
        else
            if (uav.Alive==0)
                scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);
                %scenario = Replan_Remove_DVRP(scenario, i);                
                scenario = DVRPOptimizer(scenario);
                return;
                %uav = scenario.UAVs.Value(i);
            end
        end
        
        if (uav.Alive == 0)
            continue;
        end
        
        
    else
        try
            RISK_MAP = UpdateSafeZone(RISK_MAP, uav.Position.X, uav.Position.Y);
        catch
            a = 1;
        end
        AVAILABLE_UAVS(i) = 1;
        EXPOSURE_TIME(i) = 0;
        if (lastExp == 1 &&  stealth  == 1 && uav.GroundForcesToVisit.Count()==0)
            scenario = DVRPOptimizer(scenario);
            return;
        end
    end
    
    flagVisited = 0;
    
    flightPath = uav.FlightPath;
    if flightPath.Count() > 2
        p = flightPath.First();

        if (GoalInsideAThreat(uav, flightPath.First()) == 0 || uav.Exposed == 1)
            uav.Position = flightPath.First();
        else
            uav = ApplyRRT_InC(uav, flightPath.Last());
            uav.Position = uav.FlightPath.First();
            
        end       
        uav.FlightPath.Positions = uav.FlightPath.Positions.RemoveValueInIndex(1);
        uav = uav.UpdatePathHistory();
    else
        if param.RiskMap == 1
            if (uav.WaypointsToVisit.Count() == 0)                
                goal = uav.GetNextGoalPosition(scenario);
                %RISK_MAP = UpdateRiskMap(RISK_MAP);
                [newGoal] = GetNextGoalUAVRiskMap(uav,goal);
                newGoal = newGoal.AddLast(goal);
                uav.WaypointsToVisit = newGoal;
                dist = uav.Position.CalculateDistancePos(goal);
                if dist < 100
                    idGroundForce = uav.GroundForcesToVisit.First();
                    if ~isempty(idGroundForce)
                        
                        gf = scenario.GroundForces.Value(idGroundForce);
                        gf.Visited = 1;
                        VISITED_GF(gf.Id) = 1;
                        scenario = DVRPOptimizer(scenario);
                        
                        scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);
                        uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                        
                        
                        %return;
                       
                    end
                    uav.FlightPath = FlightPath();
                else
                    
                    if (GoalInsideAThreat(uav, goal) == 0)
                        uav = ApplyRRT_InC(uav, newGoal.First());
                    else
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                        %goal = uav.WaypointsToVisit.First();
                        %uav = ApplyRRT_InC(uav, goal);
                    end
                    
                    
                end
            else
                goal = uav.WaypointsToVisit.First();
                dist = uav.Position.CalculateDistancePos(goal);
                if dist < 400
                    %idGroundForce = uav.GroundForcesToVisit.First();
                    if uav.WaypointsToVisit.Count()>0
                        %gf = scenario.GroundForces.Value(idGroundForce);
                        %gf.Visited = 1;
                        %VISITED_GF(gf.Id) = 1;
                        %scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);
                        %uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                    end
                    uav.FlightPath = FlightPath();
                else
                    if (GoalInsideAThreat(uav, goal) == 0)
                        uav = ApplyRRT_InC(uav, goal);
                    else
                        uav.WaypointsToVisit = uav.WaypointsToVisit.RemoveValueInIndex(1);
                        %goal = uav.WaypointsToVisit.First();
                        %uav = ApplyRRT_InC(uav, goal);
                    end
                    
                end
            end
        else
            goal = uav.GetNextGoalPosition(scenario);
            dist = uav.Position.CalculateDistancePos(goal);
            if dist < 50
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
%     if flagVisited == 1
%         scenario = ReOrderDVRP(scenario);
%     end
end
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