function scenario = Replan_Remove_DVRP(scenario, uav_id)
global AVAILABLE_UAVS;

nUAVs = scenario.UAVs.Count();
% Exposed UAV
uav = scenario.UAVs.Value(uav_id);
gfs = uav.GroundForcesToVisit;
uav.GroundForcesToVisit = List();
scenario.UAVs = scenario.UAVs.ModifyValueInIndex(uav_id, uav);
nGF = gfs.Count();

if (sum(AVAILABLE_UAVS) == 0)
    return;
end

currentUAV = 1;
for i = 1: nGF
    
    r = randi([1,nUAVs]);
    while(AVAILABLE_UAVS(r) == 0)
        r = randi([1,nUAVs]);
    end
    
    uav = scenario.UAVs.Value(r);
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs.Value(i));
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(r, uav);
end

scenario = ReOrderDVRP(scenario);

end