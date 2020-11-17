function [newScenario, exposed, newExposure] = DetectThreats(scenario, uavId, stealth)
    
param = Parameters();
newScenario = repmat(scenario,1);
uav = scenario.UAVs.Value(uavId);
threats = scenario.Threats;
% Check threats
detectedThreats = CheckRWR(uav, threats);
% Update exposed
uav.Exposed = detectedThreats.Count() > 0;
exposed = uav.Exposed;
uav.Exposed  = 0;
newExposure = 0;
newScenario.UAVs = newScenario.UAVs.ModifyValueInIndex(uavId, uav);  

% For each detected threat
for i = 1: detectedThreats.Count()
        
    detectedThreat = detectedThreats.Value(i);
    uav.Exposed = 1;
    newExposure = 1;        
    if (uav.Threats.Contains(detectedThreat) == 0)       
        if stealth  == 1
            %First time detect threat
            uav = CalculateGoalToLeaveThreat(uav, detectedThreat);
            detectedThreat.Detected = 1;
            uav.Threats = uav.Threats.AddLast(detectedThreat);        
            newExposure = 0;        
            ind = newScenario.Threats.IndexOf(detectedThreat);
            newScenario.Threats = newScenario.Threats.ModifyValueInIndex(ind, detectedThreat);                
            uavs = newScenario.UAVs;        
            for j = 1: uavs.Count()
                if uavId ~= j
                    if (uavs.Value(j).Exposed == 0)
                        uav2 = newScenario.UAVs.Value(j);
                        uav2.Threats = uav2.Threats.AddLast(detectedThreat);
                        uav2.FlightPath = FlightPath();
                        newScenario.UAVs = newScenario.UAVs.ModifyValueInIndex(j, uav2); 
                    end
                end
            end
        end
        ind = newScenario.Threats.IndexOf(detectedThreat);
        newScenario.Threats = newScenario.Threats.ModifyValueInIndex(ind, detectedThreat);        
    end    
    r = rand();
    if (r < param.DeathProbability)
        uav.Alive = 0;    
        uav.UnderMission = 0;
        %uav.GroundForcesToVisit = List();
    end
    newScenario.UAVs = newScenario.UAVs.ModifyValueInIndex(uavId, uav);  
    
       
end
end


function detectedThreats = CheckRWR(uav, threats)

nThreats = threats.Count();
detectedThreats = List();
for i = 1: nThreats
    threat = threats.Value(i);
    dist = uav.Position.CalculateDistancePos(threat.Position);
    if (dist <= threat.DetectionRange)
        detectedThreats = detectedThreats.AddLast(threat);
    end
end
end