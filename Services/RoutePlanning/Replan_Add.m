function scenario = Replan_Add(scenario)
global AVAILABLE_UAVS;
global VISITED_GF;

if (sum(AVAILABLE_UAVS) == 0)
    return;
end

gfs = 1:length(VISITED_GF);
[availables,~] = find(AVAILABLE_UAVS == 1);
n_availables = length(availables);
nGF = length(gfs);
gfsAvailables = ones(nGF,1);
gfsAvailables(VISITED_GF == 1) = 0;
nInt = floor(sum(gfsAvailables==1) / n_availables);

for i = 1: n_availables
    uav = scenario.UAVs.Value(availables(i));
    uav.GroundForcesToVisit = List();
    %uav.FlightPath = FlightPath();
    uav.FlightPath = uav.FlightPath.Trucante(50);
    
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(availables(i), uav);
    for j = 1: nInt
        r = randi([1,nGF]);
        while(gfsAvailables(r) == 0)
            r = randi([1,nGF]);
        end
        gfsAvailables(r) = 0;
        uav = scenario.UAVs.Value(availables(i));
        uav.UnderMission = 1;
        uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs(r));
        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(availables(i), uav);
    end
end

l = List();
for i = 1:nGF
    if  gfsAvailables(i) == 1
        l = l.AddLast(gfs(i));
    end
end
gfs = l;
nGF = l.Count();
% Exposed UAV
nUAVs = length(AVAILABLE_UAVS);
for i = 1: nGF
    
    r = randi([1,nUAVs]);
    while(AVAILABLE_UAVS(r) == 0)
        r = randi([1,nUAVs]);
    end
    %uav.FlightPath = uav.FlightPath.Trucante(50);
    uav = scenario.UAVs.Value(r);
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs.Value(i));
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(r, uav);
end

scenario = ReOrderDVRP(scenario);

end