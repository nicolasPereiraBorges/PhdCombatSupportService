
function scenario = DVRP(scenario)
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global VISITED_GF;

param = Parameters();
nGF = scenario.GroundForces.Count();
nUAVs = scenario.UAVs.Count();
EXPOSURE_TIME = zeros(nUAVs,1);
AVAILABLE_UAVS = ones(nUAVs,1);
VISITED_GF = zeros(nGF,1);

currentUAV = 1;
for i = 1: nGF
    if (currentUAV > nUAVs)
        currentUAV = 1;
    end
    
    uav = scenario.UAVs.Value(currentUAV);
    if param.ThreatsKnownPrior == 1
        uav.Threats = scenario.Threats;
        for j = 1:scenario.Threats.Count()
            t = scenario.Threats.Value(j);
            t.Detected = 1;
            scenario.Threats = scenario.Threats.ModifyValueInIndex(j, t);
        end
    end
    
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(scenario.GroundForces.Value(i).Id);
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(currentUAV, uav);
    currentUAV = currentUAV + 1;
end

scenario = ReOrderDVRP(scenario);

end

